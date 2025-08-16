#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/version.hpp>

#include <numeric>
#include <string>
#include <iostream>
#include <vector>
#include <functional>
#include <algorithm>
#include <chrono>
#include "parameter_loader.hpp"

using namespace std;
using namespace cv;

// x = m*y + b 형태의 직선 파라미터를 담는 구조체
// (일반적인 y = ax + c가 아니라 x를 y의 함수로 표현. 수직에 가까운 선을 안정적으로 표현하기 위함)
struct LineFit { float m; float b; };

class LaneDetector : public rclcpp::Node {
public:
    LaneDetector(const Config& config) 
    : Node("lane_detector_node"), config_(config),
      lane_mode_(config_.lane_mode),
      frame_width_(config_.frame_width),
      frame_height_(config_.frame_height),
      roi_top_width_(static_cast<int>(frame_width_ * config_.roi_top_width_coefficient)),
      roi_bottom_width_(static_cast<int>(frame_width_ * config_.roi_bottom_width_coefficient)),
      roi_top_y_(static_cast<int>(frame_height_ * config_.roi_top_y_coefficient)),
      roi_bottom_y_(static_cast<int>(frame_height_ * config_.roi_bottom_y_coefficient)),
      center_reference_lane_one_(config_.center_reference_lane_one),
      center_reference_lane_two_(config_.center_reference_lane_two)
    {   
        // 카메라 이미지 구독: /resized_image 토픽 (BGR8)
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/resized_image", 10,
            std::bind(&LaneDetector::imageCallback, this, std::placeholders::_1)
        );

        // 모드/레인 정보 구독: /mode_info (data = [mode, lane])
        // mode: 3=차선주행, 5=차선변경 / lane: 0=1차선, 1=2차선
        mode_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/mode_info", 10,
            std::bind(&LaneDetector::modeCallback, this, std::placeholders::_1)
        );

        // 계산된 오프셋 퍼블리셔: /lane_offset (Int16, 픽셀 단위)
        offset_pub_ = this->create_publisher<std_msgs::msg::Int16>("/lane_offset", 10);
        
        // ROI 사다리꼴을 직사각형으로 펴주는 투시변환 행렬(H)과 BEV 크기 계산
        buildHomography(); // ROI 사다리꼴 → 직사각형(BEV) 변환 행렬 계산
    }

    // ====== (1) 전처리: 노란선만 추출 ======
    // 입력: 원본 BGR 프레임
    // 출력: ROI 내부에서 "노란선 에지"만 남긴 이진(0/255) 이미지
    Mat preprocessYellow(const Mat& frame) {
        // (a) 사다리꼴 ROI 마스크 생성 및 적용 → ROI 외부는 0(검정)
        Mat roi_mask = trapezoidMask(frame.size());
        Mat roi_frame; frame.copyTo(roi_frame, roi_mask);

        // (b) 색공간 변환: HLS, HSV (노란색 검출에 유리)
        Mat hls, hsv;
        cvtColor(roi_frame, hls, COLOR_BGR2HLS);
        cvtColor(roi_frame, hsv, COLOR_BGR2HSV);

        // (c) 노란색 마스크: (HLS 범위) ∩ (HSV 범위)
        Mat y_hls, y_hsv, y_mask;
        inRange(hls,
            Scalar(config_.yellow_hls_min_h, config_.yellow_hls_min_l, config_.yellow_hls_min_s),
            Scalar(config_.yellow_hls_max_h, config_.yellow_hls_max_l, config_.yellow_hls_max_s), y_hls);

        inRange(hsv,
            Scalar(config_.yellow_hsv_min_h, config_.yellow_hsv_min_s, config_.yellow_hsv_min_v),
            Scalar(config_.yellow_hsv_max_h, config_.yellow_hsv_max_s, config_.yellow_hsv_max_v), y_hsv);

        bitwise_and(y_hls, y_hsv, y_mask);

        // (d) 모폴로지 연산으로 잡음 제거 / 끊긴 부분 연결 (Close → Open)
        Mat k_close = getStructuringElement(MORPH_RECT,
                        Size(config_.kernel_yellow_closing_size, config_.kernel_yellow_closing_size));
        Mat k_open  = getStructuringElement(MORPH_RECT,
                        Size(config_.kernel_yellow_opening_size, config_.kernel_yellow_opening_size));

        morphologyEx(y_mask, y_mask, MORPH_CLOSE, k_close);
        morphologyEx(y_mask, y_mask, MORPH_OPEN,  k_open);

        // (e) Canny 엣지: 그레이+블러 후, 색 마스크로 국소화
        Mat gray, blur_, masked_gray, edges;
        cvtColor(roi_frame, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, blur_, Size(config_.gaussian_blur_kernel_size, config_.gaussian_blur_kernel_size), 0);

        // 경계부 엣지 유실 방지: 마스크를 살짝 팽창(dilate)하여 여유를 줌
        Mat y_mask_dil;
        Mat k = getStructuringElement(MORPH_RECT, Size(3,3));
        dilate(y_mask, y_mask_dil, k);

        // 마스크 영역만 Canny 적용
        blur_.copyTo(masked_gray, y_mask_dil);
        Canny(masked_gray, edges, config_.canny_yellow_low_threshold, config_.canny_yellow_high_threshold);

        // 마지막으로 색 마스크로 한 번 더 제한 (안전)
        bitwise_and(edges, edges, edges, y_mask);

        // 결과: ROI 내부에서 노란선에 해당하는 에지 픽셀만 255로 남은 바이너리
        return edges;
    }

    // ====== (2) 투시변환: 사다리꼴 ROI → BEV (직사각형) ======
    void buildHomography() {
        // 사다리꼴의 4개 모서리(원본 좌표)
        int cx = frame_width_ / 2;
        Point2f src[4] = {
            Point2f(cx - roi_top_width_/2,    static_cast<float>(roi_top_y_)),    // 좌상
            Point2f(cx + roi_top_width_/2,    static_cast<float>(roi_top_y_)),    // 우상
            Point2f(cx + roi_bottom_width_/2, static_cast<float>(roi_bottom_y_)), // 우하
            Point2f(cx - roi_bottom_width_/2, static_cast<float>(roi_bottom_y_))  // 좌하
        };

        // BEV 결과물의 크기 설정
        // - 너비: 하단 폭(바닥에 가까운 쪽 픽셀 스케일이 신뢰도 높음)
        // - 높이: ROI의 세로 길이
        int bev_w = roi_bottom_width_;
        if (bev_w <= 0) bev_w = std::max(roi_top_width_, frame_width_);
        int bev_h = std::max(1, roi_bottom_y_ - roi_top_y_);
        bev_size_ = Size(bev_w, bev_h);

        // 직사각형 목적 좌표(좌상→우상→우하→좌하)
        Point2f dst[4] = {
            Point2f(0.f,         0.f),
            Point2f(bev_w - 1.f, 0.f),
            Point2f(bev_w - 1.f, bev_h - 1.f),
            Point2f(0.f,         bev_h - 1.f)
        };

        // 호모그래피 계산 (원본 사다리꼴 → 목적 직사각형)
        H_ = getPerspectiveTransform(src, dst);
    }

    // (사다리꼴 ROI 마스크 생성) — 전처리 단계에서 ROI 외부를 0으로 만들 때 사용
    Mat trapezoidMask(Size sz) const {
        Mat mask(sz, CV_8UC1, Scalar(0));
        int cx = frame_width_ / 2;
        Point pts[1][4] = {
            {
                Point(cx - roi_top_width_/2,    roi_top_y_),    // 좌상
                Point(cx + roi_top_width_/2,    roi_top_y_),    // 우상
                Point(cx + roi_bottom_width_/2, roi_bottom_y_), // 우하
                Point(cx - roi_bottom_width_/2, roi_bottom_y_)  // 좌하
            }
        };
        const Point* ppt[1] = { pts[0] };
        int npt[] = {4};
        fillPoly(mask, ppt, npt, 1, Scalar(255));   // 다각형 내부를 흰색(255)으로 채움
        return mask;
    }

    void drawROIPolygon(Mat& frame) const {
        int cx = frame_width_ / 2;
        // 사다리꼴 좌표
        vector<Point> pts = {
            Point(cx - roi_top_width_/2,    roi_top_y_),    // 좌상
            Point(cx + roi_top_width_/2,    roi_top_y_),    // 우상
            Point(cx + roi_bottom_width_/2, roi_bottom_y_), // 우하
            Point(cx - roi_bottom_width_/2, roi_bottom_y_)  // 좌하
        };

        // 다각형 그리기
        polylines(frame, pts, true, Scalar(0, 0, 255), 2); // 빨간색 선
        // 반투명 채우기(디버그 보기 좋게)
        Mat overlay = frame.clone();
        fillPoly(overlay, vector<vector<Point>>{pts}, Scalar(0, 0, 255));
        addWeighted(overlay, 0.2, frame, 0.8, 0, frame);
    }


    // ====== (3) BEV 상에서 슬라이딩 윈도우로 중앙선 포인트 수집/피팅 ======
    // 입력: BEV 이진 이미지(노란선 에지). ROI 외부는 이미 0.
    // 출력: x = m*y + b 형태 직선 파라미터 + ok 플래그
    LineFit fitLaneFromBEV(const Mat& bev_binary, bool& ok, cv::Mat* dbg_out = nullptr) {
        ok = false;

        // BEV 크기
        int h = bev_binary.rows, w = bev_binary.cols;

        // ★ 변경: reference 근처 코리도어(corridor) 설정
        float ref_ratio = (lane_mode_ == LaneMode::LANE_ONE) ? center_reference_lane_one_
                                                            : center_reference_lane_two_;
        int ref_x = static_cast<int>(std::round(std::clamp(ref_ratio, 0.0f, 1.0f) * w));
        int x_min = std::max(0, ref_x - static_cast<int>(config_.corridor_width / 2));
        int x_max = std::min(w - 1, ref_x + static_cast<int>(config_.corridor_width / 2));

        // (a) 시작점: 하단 1/4 히스토그램을 "ref 코리도어" 내에서만 계산
        int y_start = std::min(std::max(static_cast<int>(h * 0.3), 0), h-1); // 0.75(25%) -> 0.3(70%)
        int histW = x_max - x_min + 1;
        if (histW <= 2) { // 코리도어가 너무 좁으면 안전장치
            x_min = 0; x_max = w - 1;
            histW = w;
        }

        // === 1) 하단부 hist_roi(관심 영역)에서 세로방향 픽셀 개수(여기서는 픽셀개수 * 255 되어있음) 세기 ===
        // hist_roi: [x_min:x_max] × [y_start:h)
        cv::Mat hist_roi = bev_binary(cv::Rect(x_min, y_start, histW, h - y_start));
        cv::Mat nz = (hist_roi > 0); // (hist_roi > 0) → hist_roi 각 픽셀이 0보다 크면 255, 아니면 0을 반환
        // 각 열의 합 구하기
        cv::Mat colSum;                                  // 1×histW, CV_32S
        cv::reduce(nz, colSum, 0, cv::REDUCE_SUM, CV_32S);

        // === 2) 합 결과를 벡터로 변환 (픽셀 개수 단위로) ===
        std::vector<int> hist(histW);
        for (int i = 0; i < histW; ++i)
            hist[i] = colSum.at<int>(0, i) / 255;

        // === 3) 가장 픽셀이 많은 열의 x좌표 찾기 ===
        auto it = std::max_element(hist.begin(), hist.end()); // hist가 비어있으면, begin == end 상태가 돼서, max_element()는 그냥 first(즉, hist.end())를 그대로 반환함.
        const int bestVal = (it != hist.end()) ? *it : 0;
        int base_x = ref_x;  // 기본값은 ref_x

        // === 4) 픽셀 많은 열이 있으면 거기를 시작점으로,
        //       없으면 이전 라인 위치나 ref_x 사용
        if (bestVal > 0) {
            int base_x_rel = static_cast<int>(std::distance(hist.begin(), it));
            base_x = x_min + base_x_rel;
        } else {
            // ===== 히스토그램 무신호(엣지 케이스 1): 이전 라인 or ref_x를 시작점으로 =====
            if (has_prev_center_fit_) {
                int x_prev_bottom = static_cast<int>(prev_center_fit_.m * (h - 1) + prev_center_fit_.b);
                base_x = std::clamp(x_prev_bottom, x_min, x_max);
            } else {
                base_x = std::clamp(ref_x, x_min, x_max);
            }
        }

        // (b) 슬라이딩 윈도우 파라미터
        int num_windows = std::max(2, config_.sliding_window_num_windows);
        int margin      = std::max(5, config_.sliding_window_margin);
        size_t minpix   = std::max<size_t>(5, config_.sliding_window_minpix);
        int win_h       = h / num_windows;

        // 디버그 캔버스: 요청 있을 때만 생성(오버헤드 절약)
        cv::Mat dbg;
        if (dbg_out) {
            cv::cvtColor(bev_binary, dbg, cv::COLOR_GRAY2BGR);
            // ref/코리도어 표시
            cv::line(dbg, {ref_x,0}, {ref_x,h-1}, {0,255,0}, 1);
            for (int y=0; y<h; y+=6) {
                dbg.at<cv::Vec3b>(y, std::clamp(x_min,0,w-1)) = {255,0,0}; // std::clamp(x_min, 0, w - 1)는 배열 범위 초과 방지 (x가 음수나 w 이상이 되는 경우 방지)
                dbg.at<cv::Vec3b>(y, std::clamp(x_max,0,w-1)) = {255,0,0};
            }
            cv::line(dbg, {base_x, h-1}, {base_x, h-21}, {255,0,255}, 2);
        }

        vector<Point> pts;  // 모은 포인트들 (xx,yy)
        int x_current = base_x; // 현재 창의 중심 x

        // (c) 하단에서 상단으로 창을 올려가며, 각 창 안의 non-zero를 수집
        for (int i = 0; i < num_windows; ++i) {
            int y_low  = std::max(0,         h - (i+1)*win_h);
            int y_high = std::min(h,         h -  i   *win_h);
            
            // 코리도어 안에서만 탐색
            int xl = std::max(0, x_current - margin);
            int xr = std::min(w, x_current + margin);

            vector<int> xs; // 이번 창에서 발견된 x들의 리스트(중심 갱신용)
            for (int yy = y_low; yy < y_high; ++yy) {
                const uchar* row = bev_binary.ptr<uchar>(yy);
                for (int xx = xl; xx < xr; ++xx) {
                    if (row[xx] > 0) { pts.emplace_back(xx, yy); xs.push_back(xx); }
                }
            }
            // (d) 충분한 픽셀이 있으면, 다음 창의 중심 x를 평균값으로 이동 (드리프트 방지)
            bool recentered = false;
            if (xs.size() >= minpix) {
                // xs = 이번 윈도우 안에서 발견된 모든 픽셀들의 x좌표 목록
                // std::accumulate()-> first: 시작 반복자, last: 끝 반복자(마지막 원소 다음), init: 누적을 시작할 초기값, [first, last) 구간의 모든 원소를 차례로 꺼내서 init에 더함
                int sum = std::accumulate(xs.begin(), xs.end(), 0);
                x_current = sum / static_cast<int>(xs.size());
                recentered = true;
            }
            x_current = std::min(std::max(x_current, x_min), x_max);

            // 윈도우 시각화: 그리기만! (화면 갱신은 콜백에서 한 번)
            if (dbg_out) {
                cv::Scalar boxColor = recentered ? cv::Scalar(0,255,255) : cv::Scalar(0,165,255);
                cv::rectangle(dbg, {xl, y_low}, {xr, y_high}, boxColor, 2);
                // cv::putText(dbg, "win " + std::to_string(i) + (recentered?" ✓":" ·"),
                //             {xl+3, std::max(0,y_low-3)}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {255,255,255}, 1);
                cv::drawMarker(dbg, {x_current, (y_low+y_high)/2}, {255,255,255}, cv::MARKER_CROSS, 10, 1);
            }
        }

        // (e) 포인트가 너무 적으면 피팅 불가능 → 실패 처리
        if (pts.size() < 10) {
            RCLCPP_WARN(get_logger(), "Not enough points for fit: %zu → use previous line", pts.size());
            ok = false;
            if (dbg_out && has_prev_center_fit_) {
                float m = prev_center_fit_.m, b = prev_center_fit_.b;
                cv::line(dbg,
                        {std::clamp((int)(m*0 + b), 0, w-1), 0},
                        {std::clamp((int)(m*(h-1) + b), 0, w-1), h-1},
                        {200,200,200}, 1);
            }
            if (dbg_out) *dbg_out = std::move(dbg); // std::move() → "그림 원본을 통째로 건네주고, 내 건 빈 종이로 만든다" (빠름)
            return has_prev_center_fit_ ? prev_center_fit_ : LineFit{0.f,0.f};
        }

        // (f) 선형 회귀(최소자승(Least Squares))로 x = m*y + b 피팅
        // -------------------------------------------------------------------
        // 모델:   x = m*y + b
        // 목적:   θ = [m, b]^T 를 찾아  ||Xθ - Y||_2  최소화
        // 설계행렬 X:  N×2  (각 행은 [y_i, 1])
        // 타겟벡터 Y:  N×1  (각 원소는 x_i)
        // 해:       θ* = argmin ||Xθ - Y||_2  →  SVD 기반으로 안정적으로 구함
        // 주의:    "y = a*x + c"가 아니라 "x = m*y + b"로 피팅하는 이유는
        //          차선이 수직에 가까울 때(y축에 평행) 기울기 폭주 문제를 피하려고.
        //          (수직선에 가까워도 x=f(y)는 잘 정의됨)
        // -------------------------------------------------------------------
        Mat X(pts.size(), 2, CV_32F), Y(pts.size(), 1, CV_32F);

        // 행렬 채우기
        for (size_t i = 0; i < pts.size(); ++i) {
            float y = static_cast<float>(pts[i].y);
            X.at<float>(i,0) = y; // y값
            X.at<float>(i,1) = 1.f; // 상수항
            Y.at<float>(i,0) = static_cast<float>(pts[i].x); // x값
        }
        Mat coeff;

        // 선형 시스템 X * θ = Y 를 최소자승(Least Squares) 의미로 풂.
        // - DECOMP_SVD: SVD 기반 pseudo-inverse로 풀이 → 수치적으로 가장 안정적.
        //   * 장점: (1) X가 과결정/미결정/랭크결핍이어도 최소노름 해 제공
        //           (2) 조건수가 나쁜 데이터(스케일 격차가 큰 y)에도 비교적 강인
        // - 대안: DECOMP_NORMAL(정규방정식)은 빠르지만 수치불안정(조건수 제곱).
        // - 가중치 필요 시: W^(1/2)X, W^(1/2)Y로 전처리하여 가중 최소자승 구현 가능.
        solve(X, Y, coeff, DECOMP_SVD); // X * [m, b] = Y 를 풀어서 m, b 구하기

        // 해석: m = dx/dy(= y 방향으로 한 픽셀 올라갈 때 x가 얼마나 변하는지)
        //       b = y=0에서의 x 절편
        // 주의: 우리는 수직편차( |x̂ - x| )를 최소화하는 회귀를 하고 있음.
        //       만약 직교거리(선까지의 최단거리)를 최소화하려면 다른 기법(총 least squares 또는 cv::fitLine + 기하학적 투영)을 사용.
        LineFit fit{coeff.at<float>(0,0), coeff.at<float>(1,0)}; // m, b 저장
        
        ok = true;

        // --- 최종 라인 시각화(초록) ---
        if (dbg_out) {
            float m=fit.m,b=fit.b;
            cv::line(dbg,
                    {std::clamp((int)(m*0 + b), 0, w-1), 0},
                    {std::clamp((int)(m*(h-1) + b), 0, w-1), h-1},
                    {0,255,0}, 2);
            *dbg_out = std::move(dbg); // std::move() → "그림 원본을 통째로 건네주고, 내 건 빈 종이로 만든다" (빠름)
        }
        return fit;
    }

    // ====== (4) 오프셋 계산: 피팅한 중앙선 vs 기준선 ======
    // - y 두 지점에서의 x를 평균내서 사용 → 노이즈 평균화
    // - 기준선은 (0~1) 비율을 BEV 폭에 곱해서 픽셀로 변환해 비교
    float calcOffsetFromCenterLine(const LineFit& lf, int bev_width) const {
        // BEV의 하부 구간(근거리)을 더 신뢰 → 0.6H, 0.9H에서 샘플
        float y1 = bev_size_.height * 0.3f;
        float y2 = bev_size_.height * 0.8f;
        float x1 = lf.m * y1 + lf.b;
        float x2 = lf.m * y2 + lf.b;
        float x_mean = 0.5f * (x1 + x2);

        // 모드별 기준선 비율 선택
        float ref_ratio = (lane_mode_ == LaneMode::LANE_ONE) ? center_reference_lane_one_ : center_reference_lane_two_;
        ref_ratio = std::clamp(ref_ratio, 0.0f, 1.0f);
        float x_ref = ref_ratio * static_cast<float>(bev_width);
        
        // + 값이면 중앙선이 기준선보다 오른쪽 → 차량은 오른쪽 치우침(좌로 조향 필요)
        return (x_mean - x_ref); // +: 중앙선이 오른쪽
    }

    // (선택) 시각화: 상단에 슬라이더로 오프셋 표시
    Mat drawOffsetSlider(const Mat& bgr, float offset_px, LaneMode mode) const {
        int sw = frame_width_, sh = 50;
        Mat slider(sh, sw, CV_8UC3, Scalar(50,50,50));
        int cx = sw/2;
        line(slider, Point(cx,0), Point(cx,sh-1), Scalar(150,150,150), 1);

        int dot_x = cx + static_cast<int>(std::round(offset_px));
        dot_x = std::clamp(dot_x, 0, sw-1);
        circle(slider, Point(dot_x, sh/2), 6, Scalar(0,0,255), FILLED);

        string mode_str   = (mode == LaneMode::LANE_ONE) ? "Mode: 1-Lane" : "Mode: 2-Lane";
        string offset_str = "Offset(px): " + std::to_string(static_cast<int>(std::round(offset_px)));
        putText(slider, mode_str,   Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(220,220,220), 1);
        putText(slider, offset_str, Point(10, 42), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(220,220,220), 1);

        Mat out; vconcat(slider, bgr, out);
        return out;
    }

private:
    // ===== ROS 통신 객체 =====
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mode_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr offset_pub_;

    // ===== 파라미터 (Config에서 로드) =====
    Config config_;
    LaneMode lane_mode_;
    int frame_width_;
    int frame_height_;
    int roi_top_width_;
    int roi_bottom_width_;
    int roi_top_y_, roi_bottom_y_;
    float center_reference_lane_one_, center_reference_lane_two_;

    // ===== BEV 관련 =====
    cv::Mat H_; // 투시변환 행렬(사다리꼴→직사각형)
    cv::Size bev_size_; // BEV 결과 영상 크기
    // ★ 변경: 직전 유효 중앙선/오프셋 저장
    LineFit prev_center_fit_{0.f, 0.f};
    bool    has_prev_center_fit_ = false;
    float   prev_offset_ = 0.f;

    // 디버그 스로틀
    int frame_count_ = 0;
    int debug_stride_ = 1; // 2프레임에 한 번 그리기(원하면 Config로)
    bool debug_view_ = true; // Config가 있으면 그거 사용

    // ===== 콜백: 이미지 수신 → 전처리 → BEV → 피팅 → 오프셋 → 퍼블리시 =====
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // (0) ROS 이미지 → OpenCV Mat(BGR8)
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
            return;
        }
        Mat frame = cv_ptr->image;
        if (frame.empty()) return;

        // ROI 사다리꼴 시각화
        Mat frame_roi_vis = frame.clone();
        drawROIPolygon(frame_roi_vis);
        imshow("ROI Polygon", frame_roi_vis);

        // (1) 노란선 전처리(ROI 내부의 노란 에지 픽셀만 남김)
        Mat yellow_edges = preprocessYellow(frame);
        imshow("Mask-Yellow", yellow_edges);

        // (2) BEV 변환(사다리꼴 ROI → 직사각형). ROI 외부는 0으로 유지됨.
        Mat bev_yellow;
        warpPerspective(yellow_edges, bev_yellow, H_, bev_size_, INTER_LINEAR, BORDER_CONSTANT,  Scalar(0));
        imshow("BEV-Yellow", bev_yellow);

        // 디버그 스로틀 여부
        frame_count_++;
        bool show_dbg = debug_view_ && (frame_count_ % debug_stride_ == 0);

        // (3) BEV 상에서 슬라이딩 윈도우로 중앙선 포인트 수집 → 직선 피팅(x = m*y + b)
        bool valid = false;
        cv::Mat dbg;
        LineFit center_fit = fitLaneFromBEV(bev_yellow, valid, show_dbg ? &dbg : nullptr);

        // (4) 기준선 대비 오프셋 계산 (픽셀)
        float offset = 0.f;
        if (valid) {
            offset = calcOffsetFromCenterLine(center_fit, bev_size_.width);
            prev_offset_ = offset;               // 직전 offset 갱신
            prev_center_fit_ = center_fit;          // 직전 라인 갱신
            has_prev_center_fit_ = true;
        } else {
            if (has_prev_center_fit_) {
                offset = prev_offset_;           // 직전 offset 재사용
                RCLCPP_INFO(get_logger(), "Center lane invalid → reuse previous offset: %.1f", offset);
            } else {
                // 처음부터 데이터가 없으면 0 사용
                offset = 0.f;
                RCLCPP_INFO(get_logger(), "Center lane invalid and no history → offset=0");
            }
        }

        // (5) 퍼블리시: /lane_offset (Int16)
        std_msgs::msg::Int16 offset_msg;
        offset_msg.data = static_cast<int16_t>(std::round(offset));
        offset_pub_->publish(offset_msg);

        // === 디버그는 프레임 당 딱 한 번 ===
        if (show_dbg) {
            // 원하면 필요한 것만 보여줘: 창 1~2개로 제한
            // 1) 슬라이딩윈도 디버그
            if (!dbg.empty()) cv::imshow("SlidingWindows", dbg);

            // 2) 오프셋 슬라이더
            Mat vis = drawOffsetSlider(frame, offset, lane_mode_);
            cv::imshow("Lane View + Offset", vis);

            // 3) ROI/Mask/BEV는 필요할 때만 (주석 권장)
            Mat frame_roi_vis = frame.clone(); drawROIPolygon(frame_roi_vis); imshow("ROI Polygon", frame_roi_vis);
            imshow("Mask-Yellow", yellow_edges);
            imshow("BEV-Yellow", bev_yellow);

            cv::waitKey(1); // 한 번만!
        }
    }

    // ===== 콜백: 모드/레인 변경 =====
    void modeCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        // 입력 형식 가정: data = [mode, lane], lane: 0=1차선, 1=2차선
        if (msg->data.size() >= 2 && msg->data[0] == 5){ // 차선 변경 모드 시 레인 갱신
            lane_mode_ = (msg->data[1] == 0) ? LaneMode::LANE_ONE : LaneMode::LANE_TWO;
        }
    }
};

int main(int argc, char** argv) {
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
    rclcpp::init(argc, argv);

    // JSON 경로는 환경에 맞게 변경
    Config config = load_config(
        "/home/xytron/xycar_ws/src/orda/modular/lane_detection/lane_detection_parameter.json"
    );

    auto node = std::make_shared<LaneDetector>(config);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}