#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
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

// ======================== 직선 모델 구조체 ========================
// 직선: x = m*y + b 형태
// - 일반적인 y = ax + c 대신 x=f(y) 꼴을 사용
//   (세로에 가까운 차선을 안정적으로 표현 가능)
struct LineFit { float m; float b; };

// ======================== LaneDetector 클래스 ========================
class LaneDetector : public rclcpp::Node {
public:
    // -------------------- 생성자 --------------------
    // - Config 파일에서 로드한 파라미터 기반 초기화
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
        // 이미지/센서용: 최신성 중시, 유실 허용
        auto qos_sensor = rclcpp::SensorDataQoS().best_effort();   // KeepLast(depth)는 SensorDataQoS 기본
        // 가벼운 수치 토픽용: BestEffort + Volatile (latched 아님)
        auto qos_fast   = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        // 상태/명령은 유실이 치명적이면 Reliable 유지 권장
        auto qos_state  = rclcpp::QoS(rclcpp::KeepLast(10));

        // 1) 카메라 영상 구독: /resized_image (BGR8)
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/resized_image", qos_sensor,
            std::bind(&LaneDetector::imageCallback, this, std::placeholders::_1)
        );

        // 2) 모드/차선 정보 구독: /mode_info
        //    - mode: 3=차선주행, 5=차선변경
        //    - lane: 0=1차선, 1=2차선
        mode_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/mode_info", qos_fast,
            std::bind(&LaneDetector::modeCallback, this, std::placeholders::_1)
        );

        // 3) 계산된 오프셋 발행: /lane_offset (Int16, 픽셀 단위)
        offset_pub_ = this->create_publisher<std_msgs::msg::Int16>("/lane_offset", qos_fast);
        
        // 4) 계산된 중앙선 파라미터 발행: /lane_fit (Float32MultiArray, [m, b])
        fit_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/lane_fit", qos_fast);

        // 5) (차선변경모드인지, 차선변경성공했는지) 를 담아서 발행한다.
        lane_change_state_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/lane_change_state", qos_fast);

        // 6) ROI 사다리꼴을 BEV 직사각형으로 변환하기 위한 호모그래피 행렬 계산
        buildHomography();

        // lane ref 전환 시간: config에 있으면 사용(없거나 <=0면 기본 0.8초)
        smooth_enabled_ = config_.change_ref_smoothly; // ← 추가
        if (config_.lane_ref_transition_duration_sec > 0.0)
            ref_transition_duration_sec_ = config_.lane_ref_transition_duration_sec;

        // 초기 ref 비율은 현재 lane_mode_ 기준의 목표값으로 설정
        if (smooth_enabled_) {
            ref_ratio_current_ = getTargetRefForMode(lane_mode_);
            ref_ratio_start_   = ref_ratio_current_;
            ref_ratio_target_  = ref_ratio_current_;
            ref_start_time_    = this->now();
            ref_transition_active_ = false;
        }
    }

    // ====================================================================
    // (1) 전처리 단계: 영상에서 "노란 차선 에지"만 추출
    // ====================================================================
    Mat preprocessYellow(const Mat& frame) {
        // --- (a) ROI 마스크 ---
        Mat roi_mask = trapezoidMask(frame.size());
        Mat roi_frame; frame.copyTo(roi_frame, roi_mask);

        // --- (b) 색공간 변환: HLS, HSV, YCrCb ---
        Mat hls, hsv, ycrcb;
        cvtColor(roi_frame, hls,  COLOR_BGR2HLS);
        cvtColor(roi_frame, hsv,  COLOR_BGR2HSV);
        cvtColor(roi_frame, ycrcb, COLOR_BGR2YCrCb); // OpenCV 순서: [Y, Cr, Cb]

        // --- (c) 노란색 범위 마스크 생성 (HLS ∩ HSV ∩ YCrCb) ---
        Mat y_hls, y_hsv, y_ycc, y_mask;
        inRange(
            hls,
            Scalar(config_.yellow_hls_min_h, config_.yellow_hls_min_l, config_.yellow_hls_min_s),
            Scalar(config_.yellow_hls_max_h, config_.yellow_hls_max_l, config_.yellow_hls_max_s),
            y_hls
        );

        inRange(
            hsv,
            Scalar(config_.yellow_hsv_min_h, config_.yellow_hsv_min_s, config_.yellow_hsv_min_v),
            Scalar(config_.yellow_hsv_max_h, config_.yellow_hsv_max_s, config_.yellow_hsv_max_v),
            y_hsv
        );

        // YCrCb: 채널 순서 [Y, Cr, Cb] 임에 주의!
        inRange(
            ycrcb,
            Scalar(config_.yellow_ycrcb_min_y,  config_.yellow_ycrcb_min_cr, config_.yellow_ycrcb_min_cb),
            Scalar(config_.yellow_ycrcb_max_y,  config_.yellow_ycrcb_max_cr, config_.yellow_ycrcb_max_cb),
            y_ycc
        );
        // ycrcb 시각화
        if (config_.debug_view) {
            std::vector<cv::Mat> ch; split(ycrcb, ch);
            imshow("YCrCb channels", (ch[0] + ch[1] + ch[2]) / 3);
        }   
        
        // 교집합
        bitwise_and(y_hls, y_hsv, y_mask);
        bitwise_and(y_mask, y_ycc, y_mask);

        // (선택) 아주 어두운 픽셀 억제 게이트(검정 재유입 방지)
        // cv::Mat y_gate = (ycrcb.channels()==3 ? (ycrcb[:,:,0] > Y_FLOOR) : cv::Mat());
        // 여기선 단순히 Y 바닥선 5 적용
        {
            std::vector<cv::Mat> ch; split(ycrcb, ch);
            cv::Mat y_gate = ch[0] > 5; // Y > 5
            y_gate.convertTo(y_gate, CV_8U, 255);
            bitwise_and(y_mask, y_gate, y_mask);
        }

        // --- (d) 모폴로지 ---
        Mat k_close = getStructuringElement(MORPH_RECT,
                        Size(config_.kernel_yellow_closing_size, config_.kernel_yellow_closing_size));
        Mat k_open  = getStructuringElement(MORPH_RECT,
                        Size(config_.kernel_yellow_opening_size,  config_.kernel_yellow_opening_size));

        morphologyEx(y_mask, y_mask, MORPH_CLOSE, k_close);
        morphologyEx(y_mask, y_mask, MORPH_OPEN,  k_open);

        // --- (e) Canny ---
        Mat gray, blur_, masked_gray, edges;
        cvtColor(roi_frame, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, blur_, Size(config_.gaussian_blur_kernel_size, config_.gaussian_blur_kernel_size), 0);

        Mat y_mask_dil;
        Mat k = getStructuringElement(MORPH_RECT, Size(3,3));
        dilate(y_mask, y_mask_dil, k);

        blur_.copyTo(masked_gray, y_mask_dil);
        Canny(masked_gray, edges, config_.canny_yellow_low_threshold, config_.canny_yellow_high_threshold);

        bitwise_and(edges, edges, edges, y_mask);
        return edges;
    }


    // ====================================================================
    // (2) 투시변환: ROI 사다리꼴 → BEV 직사각형
    // ====================================================================
    void buildHomography() {
        // 사다리꼴의 4개 모서리(원본 좌표)
        int cx = frame_width_ / 2;

        // --- (a) 원본 사다리꼴 좌표 ---
        Point2f src[4] = {
            Point2f(cx - roi_top_width_/2,    static_cast<float>(roi_top_y_)),    // 좌상
            Point2f(cx + roi_top_width_/2,    static_cast<float>(roi_top_y_)),    // 우상
            Point2f(cx + roi_bottom_width_/2, static_cast<float>(roi_bottom_y_)), // 우하
            Point2f(cx - roi_bottom_width_/2, static_cast<float>(roi_bottom_y_))  // 좌하
        };

        // --- (b) BEV 결과 크기 ---
        int bev_w = roi_bottom_width_;
        if (bev_w <= 0) bev_w = std::max(roi_top_width_, frame_width_);
        int bev_h = std::max(1, roi_bottom_y_ - roi_top_y_);
        bev_size_ = Size(bev_w, bev_h);

        // --- (c) 직사각형 목적 좌표 ---
        Point2f dst[4] = {
            Point2f(0.f,         0.f),
            Point2f(bev_w - 1.f, 0.f),
            Point2f(bev_w - 1.f, bev_h - 1.f),
            Point2f(0.f,         bev_h - 1.f)
        };

        // --- (d) 호모그래피 행렬 계산 ---
        H_ = getPerspectiveTransform(src, dst);
        H_inv_ = H_.inv();  // ★ 추가: 역변환 미리 구해둠
    }

    // ====================================================================
    // (3) 수평 노이즈 검출 및 제거
    // - BEV 이진 영상에서 "가로로 길게 이어진 흰색"을 찾아서 노이즈 여부 판정
    // - band_h 줄을 세로로 OR(합치기)해서, 약간 기울어진(대각선) 것도 잡을 수 있음
    // - 노이즈 줄을 0으로 만들면서, vis가 있으면 그 줄을 색으로 칠해 표시
    // ====================================================================
    bool suppressHorizontalNoiseRows(const cv::Mat& bev_in,
                                 cv::Mat& bev_out,
                                 int horizontal_noise_width,
                                 float corridor_ratio_thresh, // ★ 추가: 비율 임계값(0~1)
                                 int y_start,
                                 int y_end,
                                 int band_h,
                                 int extra_pad_rows,      // 위/아래 추가 삭제 줄
                                 int x_min,               // ★ corridor 범위 시작 x
                                 int x_max,               // ★ corridor 범위 끝   x (포함)
                                 cv::Mat* vis /* = nullptr */)
    {
        CV_Assert(bev_in.type() == CV_8UC1);
        const int H = bev_in.rows, W = bev_in.cols;

        bev_out = bev_in.clone();

        // 경계 보정
        y_start = std::max(0, y_start);
        y_end   = (y_end <= 0 || y_end > H) ? H : y_end;
        band_h  = std::max(1, band_h);
        extra_pad_rows = std::max(0, extra_pad_rows);
        x_min = std::max(0, x_min);
        x_max = std::min(W - 1, x_max);
        if (y_start >= y_end || x_min > x_max) return false;

        const int corridor_w = x_max - x_min + 1;
        const int corridor_h = y_end - y_start;

        // ★ corridor 전체 흰 픽셀 수(정규화 분모) 미리 계산
        int corridor_total_whites = 0;
        if (corridor_w > 0 && corridor_h > 0) {
            cv::Mat corridor_roi = bev_in(cv::Rect(x_min, y_start, corridor_w, corridor_h));
            corridor_total_whites = cv::countNonZero(corridor_roi);
        }

        bool suppressed_any = false;

        // y를 한 줄씩 올리며 [y..y+band_h-1]을 한 줄처럼 보고 run-length 측정
        for (int y = y_start; y <= y_end - band_h; ++y) {
            int best = 0, run = 0;

            // band_h 줄을 세로 OR한 가상의 한 줄에서 run-length 측정
            // corridor 범위 [x_min..x_max]에서만 판별
            for (int x = x_min; x < x_max; ++x) {
                // 이 x에서, 세로로 band_h 줄 중 하나라도 흰 픽셀이 있으면 1로 간주
                bool on = false;
                for (int dy = 0; dy < band_h; ++dy) {
                    if (bev_in.at<uchar>(y + dy, x) > 0) { on = true; break; }
                }

                if (on) { run++; best = std::max(best, run); }
                else    { run = 0; }
            }

            // ★ 비율 조건: (밴드 내 흰 픽셀 수 / corridor 전체 흰 픽셀 수) >= 임계값
            bool ratio_noise = false;
            if (corridor_total_whites > 0) {
                cv::Mat band_roi = bev_in(cv::Rect(x_min, y, corridor_w, band_h));
                int band_whites = cv::countNonZero(band_roi);
                float ratio = static_cast<float>(band_whites) / static_cast<float>(corridor_total_whites);
                ratio_noise = (ratio >= corridor_ratio_thresh);
            }

            // 이 병합줄에서 "가로로 충분히 긴 스트릭"이 있으면 카운트
            if (best >= horizontal_noise_width && ratio_noise) {
                // ★ 패딩 포함해서 실제로 지울 y-범위 계산
                const int y0 = std::max(y_start, y - extra_pad_rows);
                const int y1 = std::min(H, y + band_h + extra_pad_rows); // [y0, y1) 지움

                // corridor 구간만 삭제
                for (int yy = y0; yy < y1; ++yy) {
                    uchar* row = bev_out.ptr<uchar>(yy);
                    std::memset(row + x_min, 0, (x_max - x_min + 1));
                }

                // 시각화(있으면)도 corridor만 채움
                if (vis && !vis->empty()) {
                    cv::rectangle(*vis,
                                cv::Rect(x_min, y0, x_max - x_min + 1, y1 - y0),
                                cv::Scalar(0, 0, 255), cv::FILLED);
                }

                suppressed_any = true;

                // ★ 스캔 건너뛰기: 패딩 끝 다음 위치로 점프(중복 검출/지우기 방지)
                //   다음 루프에서 y++ 되므로 -1 보정
                y = std::min(y1, y_end - band_h + 1) - 1;
            }
        }

        return suppressed_any;
    }

    // ====================================================================
    // 열 밴드 기반 노이즈 억제
    //
    // - BEV 이진 영상에서 여러 열을 band_w 폭으로 묶어서 검사
    // - band 폭 안에서 "세로 픽셀 합"이 충분히 크면 → 노이즈 밴드로 판단
    // - 밴드 전체(x축 band 범위, y_start~y_end)와 양쪽 half_w 여유까지 0으로 지움
    // - 디버그 그림(vis)이 있으면 빨간색으로 칠함
    //
    // 입력:
    //   bev_in     : BEV 이진영상
    //   x_min~x_max: 검사할 가로 범위
    //   y_start~y_end: 검사할 세로 범위
    //   band_w     : 한 번에 묶어볼 열 밴드 폭
    //   min_pixels : 밴드 내 최소 픽셀 수 (절대 기준)
    //   peak_ratio : 최대 피크 대비 비율 기준 (0~1)
    //   half_w     : 밴드 주변 확장 폭
    // ====================================================================
    bool suppressColumnBands(const cv::Mat& bev_in,
                            cv::Mat& bev_out,
                            int x_min, int x_max,
                            int y_start, int y_end,
                            int band_w,
                            int min_pixels,
                            float peak_ratio,
                            int half_w,
                            cv::Mat* vis /*=nullptr*/)
    {
        CV_Assert(bev_in.type() == CV_8UC1);
        const int H = bev_in.rows, W = bev_in.cols;

        // 경계 보정
        x_min   = std::max(0, x_min);
        x_max   = std::min(W - 1, x_max);
        y_start = std::max(0, y_start);
        y_end   = (y_end <= 0 || y_end > H) ? H : y_end;
        band_w  = std::max(1, band_w);
        half_w  = std::max(0, half_w);

        bev_out = bev_in.clone();
        if (x_min > x_max || y_start >= y_end) return false;

        // (1) ROI 영역 세로 합
        cv::Mat roi = bev_in(cv::Rect(x_min, y_start, x_max - x_min + 1, y_end - y_start));
        cv::Mat nz = (roi > 0);
        cv::Mat colSum32S;
        cv::reduce(nz, colSum32S, 0, cv::REDUCE_SUM, CV_32S); // (1 x width)

        // (2) maxVal 찾기
        int maxVal = 0;
        for (int i=0; i<colSum32S.cols; ++i)
            maxVal = std::max(maxVal, colSum32S.at<int>(0,i)/255);

        bool any = false;

        // (3) band_w 폭 단위로 검사
        for (int i=0; i<colSum32S.cols; i += band_w) {
            int x_band_start = x_min + i;
            int x_band_end   = std::min(x_max, x_band_start + band_w - 1);

            // 밴드 내 픽셀 합
            int bandPix = 0;
            for (int j=i; j< i+band_w && j<colSum32S.cols; ++j) {
                bandPix += colSum32S.at<int>(0,j) / 255;
            }

            // 절대 기준 + 비율 기준
            bool cond_abs = (bandPix >= min_pixels);
            bool cond_rel = (peak_ratio > 0.f) ? (bandPix >= maxVal * peak_ratio) : true;

            if (cond_abs && cond_rel) {
                // 삭제 구간: 밴드 + 좌우 half_w
                int xl = std::max(0, x_band_start - half_w);
                int xr = std::min(W-1, x_band_end + half_w);

                for (int yy=y_start; yy<y_end; ++yy) {
                    uchar* row = bev_out.ptr<uchar>(yy);
                    std::memset(row + xl, 0, (xr - xl + 1));
                }

                if (vis && !vis->empty()) {
                    cv::rectangle(*vis,
                                cv::Rect(xl, y_start, xr - xl + 1, y_end - y_start),
                                cv::Scalar(0,0,255), cv::FILLED);
                }

                any = true;
            }
        }

        return any;
    }

    // ====================================================================
    // (4) BEV 상에서 슬라이딩 윈도우로 중앙선 포인트 수집 → 직선 피팅
    //  - 입력:  bev_binary (CV_8UC1, BEV에서의 이진 에지 영상)
    //  - 출력:  LineFit { m, b }  (x = m*y + b)
    //  - ok   : true=정상 피팅, false=실패(이전 라인/레퍼런스 사용 권장)
    //  - dbg_out: 디버그용 캔버스 요청 시, corridor/윈도우/마커/최종 선을 그려서 반환
    //  - 이유: x=f(y) 꼴은 수직에 가까운 선도 안정적으로 표현 가능
    // ====================================================================
    LineFit fitLaneFromBEV(const Mat& bev_binary, bool& ok, cv::Mat* dbg_out = nullptr) {
        ok = false;

        // ---------- 0) 기초 정보 ----------
        int h = bev_binary.rows, w = bev_binary.cols;

        // ---------- 1) reference 기반 가로 "코리도어(corridor)" 설정 ----------
        //  - 목적: 검색 범위를 기준선 주변으로 제한하여 오탐/계산량 감소
        //  - ref_ratio: 모드별 기준선 비율(0~1), BEV 폭에 곱해 픽셀 x좌표로 변환
        // float ref_ratio = (lane_mode_ == LaneMode::LANE_ONE) ? center_reference_lane_one_
        //                                                     : center_reference_lane_two_;
        // ref_x_ = static_cast<int>(std::round(std::clamp(ref_ratio, 0.0f, 1.0f) * w));
        float ref_ratio = std::clamp(getActiveRefRatio(), 0.0f, 1.0f);
        ref_x_ = static_cast<int>(std::round(ref_ratio * w));

        // corridor 좌우 경계 (폭은 config_.corridor_width)
        int x_min = std::max(0, ref_x_ - static_cast<int>(config_.corridor_width / 2));
        int x_max = std::min(w - 1, ref_x_ + static_cast<int>(config_.corridor_width / 2));

        // corridor 폭이 비정상적으로 작으면 전체 폭으로 안전하게 확장
        int histW = x_max - x_min + 1;
        if (histW <= 2) { x_min = 0; x_max = w - 1; histW = w; }

        // ---------- 2) 시작점 추출(히스토그램) ----------
        //  - 하단부(근거리)만 신뢰도가 높으므로, 아래쪽 일부(y_start~h-1)에서
        //    corridor 영역 [x_min, x_max]의 "세로 합"이 가장 큰 열을 시작점으로 선택
        //  - 히스토그램이 빈약하면 이전 프레임의 추정(or ref_x_)을 사용
        int y_start = std::min(std::max(static_cast<int>(h * 0.3), 0), h-1); // 하단 70% 사용
        cv::Mat hist_roi = bev_binary(cv::Rect(x_min, y_start, histW, h - y_start));
        cv::Mat nz = (hist_roi > 0); // (hist_roi > 0) → hist_roi 각 픽셀이 0보다 크면 255, 아니면 0을 반환
        cv::Mat colSum;
        cv::reduce(nz, colSum, 0, cv::REDUCE_SUM, CV_32S); // 열(세로) 합계 (1 x histW)

        // (a) 기본 히스토그램(픽셀 개수)
        std::vector<int> hist(histW);
        for (int i = 0; i < histW; ++i) hist[i] = colSum.at<int>(0, i) / 255; // 실제 픽셀 개수로 변환
        
        // (b) ref_x_ 근처에 더 큰 가중치 부여
        //    - 파라미터: 없으면 아래 기본값 사용
        // sigma_ratio가 작을수록 ref 근처만 강하게 밀어줌(보수적 시작점).
        // w_min을 0.3~0.7 사이로 조절하며 “멀리 있어도 완전 무시되지는 않게” 균형 잡아줘.
        const float sigma_ratio   = (config_.ref_hist_sigma_ratio > 0.f) ? 
                                    config_.ref_hist_sigma_ratio : 0.35f; // corridor 폭 대비 σ
        const float w_min         = (config_.ref_hist_min_weight > 0.f && config_.ref_hist_min_weight < 1.f) ?
                                    config_.ref_hist_min_weight : 0.50f;  // 최소 가중치 (0~1)
        const float sigma_pixels  = std::max(1.f, sigma_ratio * static_cast<float>(std::max(1, x_max - x_min + 1)));
        const float two_sigma2    = 2.f * sigma_pixels * sigma_pixels;

        // (c) 가중 히스토그램 계산
        std::vector<float> weighted_hist(histW);
        for (int i = 0; i < histW; ++i) {
            int x = x_min + i;
            float d = static_cast<float>(std::abs(x - ref_x_));
            float w = w_min + (1.f - w_min) * std::exp(-(d*d) / two_sigma2);
            weighted_hist[i] = static_cast<float>(hist[i]) * w;
        }

        // (d) 최댓값 찾기 (가중치 적용 후)
        int base_x = ref_x_;
        auto it_w = std::max_element(weighted_hist.begin(), weighted_hist.end());
        const float bestValW = (it_w != weighted_hist.end()) ? *it_w : 0.f;

        if (bestValW > 0.f) {
            int base_x_rel = static_cast<int>(std::distance(weighted_hist.begin(), it_w));
            base_x = x_min + base_x_rel;
        } else {
            // 신호가 없으면 이전 프레임 or ref_x_ 사용(기존 로직 유지)
            if (has_prev_center_fit_) {
                int x_prev_bottom = static_cast<int>(prev_center_fit_.m * (h - 1) + prev_center_fit_.b);
                base_x = std::clamp(x_prev_bottom, x_min, x_max);
            } else {
                base_x = std::clamp(ref_x_, x_min, x_max);
            }
        }

        // ---------- 3) 디버그 캔버스 준비 ----------
        //  - corridor의 좌/우 가이드를 점선처럼 표시
        //  - ref_x_(초록 실선), base_x(보라색 작은 세그먼트) 표시
        cv::Mat dbg;
        if (dbg_out) {
            cv::cvtColor(bev_binary, dbg, cv::COLOR_GRAY2BGR);
            
            // ref_x_ (초록)
            cv::line(dbg, {ref_x_,0}, {ref_x_,h-1}, {0,255,0}, 1);

            // corridor 좌우 경계 (파랑 점선 느낌)
            for (int y=0; y<h; y+=6) {
                dbg.at<cv::Vec3b>(y, std::clamp(x_min,0,w-1)) = {255,0,0}; // std::clamp(x_min, 0, w - 1)는 배열 범위 초과 방지 (x가 음수나 w 이상이 되는 경우 방지)
                dbg.at<cv::Vec3b>(y, std::clamp(x_max,0,w-1)) = {255,0,0};
            }

            // base_x (보라색 작은 막대)
            cv::line(dbg, {base_x, h-1}, {base_x, h-21}, {255,0,255}, 2);
        }

        // ---------- 4) 슬라이딩 윈도우로 포인트 수집 ----------
        //  - 아래에서 위로 창을 올리며, 각 창 내 non-zero 픽셀을 수집
        //  - 충분한 픽셀(minpix) 발견 시, 다음 창의 중심을 평균 x로 재조정(recenter)
        int num_windows = std::max(2, config_.sliding_window_num_windows);
        int margin      = std::max(5, config_.sliding_window_margin);
        size_t minpix   = std::max<size_t>(5, config_.sliding_window_minpix);
        int win_h       = h / num_windows;

        vector<Point> pts;  // 모은 포인트들 (xx,yy)
        int x_current = base_x; // 현재 창의 중심 x

        for (int i = 0; i < num_windows; ++i) {
            int y_low  = std::max(0,         h - (i+1)*win_h);
            int y_high = std::min(h,         h -  i   *win_h);
            
            // corridor 내부에서만 검색
            int xl = std::max(0, x_current - margin);
            int xr = std::min(w, x_current + margin);

            vector<int> xs; // 이번 창에서 수집한 픽셀들의 x (중심 재조정용)
            for (int yy = y_low; yy < y_high; ++yy) {
                const uchar* row = bev_binary.ptr<uchar>(yy);
                for (int xx = xl; xx < xr; ++xx) {
                    if (row[xx] > 0) { pts.emplace_back(xx, yy); xs.push_back(xx); }
                }
            }
            
            // 픽셀이 충분하면 창 중심 x를 평균값으로 이동 (드리프트 방지)
            bool recentered = false;
            if (xs.size() >= minpix) {
                // xs = 이번 윈도우 안에서 발견된 모든 픽셀들의 x좌표 목록
                // std::accumulate()-> first: 시작 반복자, last: 끝 반복자(마지막 원소 다음), init: 누적을 시작할 초기값, [first, last) 구간의 모든 원소를 차례로 꺼내서 init에 더함
                int sum = std::accumulate(xs.begin(), xs.end(), 0);
                x_current = sum / static_cast<int>(xs.size());
                recentered = true;
            }
            x_current = std::min(std::max(x_current, x_min), x_max);

            // 디버그: 현재 윈도우 박스 + 중심 마커
            if (dbg_out) {
                cv::Scalar boxColor = recentered ? cv::Scalar(0,255,255) : cv::Scalar(0,165,255);
                cv::rectangle(dbg, {xl, y_low}, {xr, y_high}, boxColor, 2);
                // cv::putText(dbg, "win " + std::to_string(i) + (recentered?" ✓":" ·"),
                //             {xl+3, std::max(0,y_low-3)}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {255,255,255}, 1);
                cv::drawMarker(dbg, {x_current, (y_low+y_high)/2}, {255,255,255}, cv::MARKER_CROSS, 10, 1);
            }
        }

        // ---------- 5) 포인트 수가 부족하면 실패 처리 ----------
        if (pts.size() < 10) {
            // RCLCPP_WARN(get_logger(), "Not enough points for fit: %zu → use previous line", pts.size());
            ok = false;
            if (dbg_out && has_prev_center_fit_) {
                // 이전 라인을 희미하게 표시(회색)
                float m = prev_center_fit_.m, b = prev_center_fit_.b;
                cv::line(dbg,
                        {std::clamp((int)(m*0 + b), 0, w-1), 0},
                        {std::clamp((int)(m*(h-1) + b), 0, w-1), h-1},
                        {200,200,200}, 1);
            }
            if (dbg_out) *dbg_out = std::move(dbg); // std::move() → "그림 원본을 통째로 건네주고, 내 건 빈 종이로 만든다" (빠름)
            return has_prev_center_fit_ ? prev_center_fit_ : LineFit{0.f,0.f};
        }

        // ---------- 6) 최소자승 직선 피팅: x = m*y + b ----------
        //  - 설계행렬 X(Nx2): 각 행 [y_i, 1]
        //  - 타겟벡터 Y(Nx1): 각 원소 x_i
        //  - 해: θ = [m, b] (SVD를 이용해 수치적으로 안정적으로 계산)
        Mat X(pts.size(), 2, CV_32F), Y(pts.size(), 1, CV_32F);

        // 행렬 채우기
        for (size_t i = 0; i < pts.size(); ++i) {
            float y = static_cast<float>(pts[i].y);
            X.at<float>(i,0) = y; // y값
            X.at<float>(i,1) = 1.f; // 상수항
            Y.at<float>(i,0) = static_cast<float>(pts[i].x); // x값
        }
        Mat coeff;
        solve(X, Y, coeff, DECOMP_SVD); // X * [m, b] = Y 를 풀어서 m, b 구하기

        LineFit fit{coeff.at<float>(0,0), coeff.at<float>(1,0)}; // m, b 저장        
        ok = true;

        // ---------- 7) 최종 선 디버그 표시 ----------
        if (dbg_out) {
            float m=fit.m,b=fit.b;
            cv::line(dbg,
                    {std::clamp((int)(m*0 + b), 0, w-1), 0},
                    {std::clamp((int)(m*(h-1) + b), 0, w-1), h-1},
                    {0,255,0}, 2); // 최종 회귀선(초록)
            *dbg_out = std::move(dbg); // std::move() → "그림 원본을 통째로 건네주고, 내 건 빈 종이로 만든다" (빠름)
        }
        return fit;
    }

    // ====================================================================
    // (5) 기준선 대비 오프셋 계산
    //  - 입력 : lf         → 중앙선 회귀 결과 (x = m*y + b)
    //          bev_width  → BEV 이미지의 가로폭(px)
    //  - 동작 : BEV 하부(근거리) 두 지점(y1, y2)에서 중앙선 x를 샘플링하여 평균 x_mean 계산
    //          모드별 기준선(ref_ratio ∈ [0,1])을 픽셀로 변환한 x_ref와의 차이를 오프셋으로 반환
    //  - 출력 : +값 → 중앙선이 기준선보다 "오른쪽"(차량은 오른쪽 치우침 → 좌로 조향 필요)
    //           -값 → 중앙선이 기준선보다 "왼쪽"
    //  - 비고 : 하부 비중을 높여(0.3H, 0.8H) 원근/노이즈 영향 완화
    // ====================================================================
    float calcOffsetFromCenterLine(const LineFit& lf, int bev_width) const {
        // 1) 샘플링 y (BEV 높이의 비율로 지정: 근거리 위주)
        float y1 = bev_size_.height * 0.3f; // 하부 30% 지점
        float y2 = bev_size_.height * 0.8f; // 하부 80% 지점

        // 2) 중앙선 x = m*y + b 에서 두 지점의 x값 추정
        float x1 = lf.m * y1 + lf.b;
        float x2 = lf.m * y2 + lf.b;

        // 3) 평균 x (노이즈/기울기 영향 완화용)
        float x_mean = 0.5f * (x1 + x2);

        // 4) 모드별 기준선(ref_ratio)을 픽셀로 변환 → x_ref
        // float ref_ratio = (lane_mode_ == LaneMode::LANE_ONE) ? center_reference_lane_one_ : center_reference_lane_two_;
        // ref_ratio = std::clamp(ref_ratio, 0.0f, 1.0f);
        // float x_ref = ref_ratio * static_cast<float>(bev_width);
        
        float ref_ratio = std::clamp(getActiveRefRatio(), 0.0f, 1.0f);
        float x_ref = ref_ratio * static_cast<float>(bev_width);

        // 5) 오프셋(픽셀) 계산
        //    +면 중앙선이 기준선보다 오른쪽, -면 왼쪽
        return (x_mean - x_ref); // +: 중앙선이 오른쪽
    }

    // === 사다리꼴 ROI 마스크 생성 ===
    // - 입력 크기(sz)와 사다리꼴 좌표(roi_* 변수들)를 이용해
    //   관심영역(ROI)을 흰색(255)으로 표시한 마스크(Mat)를 반환
    Mat trapezoidMask(Size sz) const {
        Mat mask(sz, CV_8UC1, Scalar(0)); // 전체 0(검은색)으로 초기화
        int cx = frame_width_ / 2; // 화면 중앙 x 좌표
        Point pts[1][4] = { // 사다리꼴 꼭짓점 4개
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

    // === ROI 사다리꼴 디버그 출력 ===
    // - 원본 프레임(frame)에 ROI 사다리꼴 영역을 그려서 확인용으로 사용
    // - 외곽선(빨간색) + 반투명 채우기
    void drawROIPolygon(Mat& frame) const {
        int cx = frame_width_ / 2; // 화면 중앙 x 좌표
        // 사다리꼴 꼭짓점
        vector<Point> pts = {
            Point(cx - roi_top_width_/2,    roi_top_y_),    // 좌상
            Point(cx + roi_top_width_/2,    roi_top_y_),    // 우상
            Point(cx + roi_bottom_width_/2, roi_bottom_y_), // 우하
            Point(cx - roi_bottom_width_/2, roi_bottom_y_)  // 좌하
        };

        // 외곽선(빨간색, 두께 2)
        polylines(frame, pts, true, Scalar(0, 0, 255), 2); // 빨간색 선
        
        // 반투명 빨간색 채우기 (디버그 시각화용)
        Mat overlay = frame.clone();
        fillPoly(overlay, vector<vector<Point>>{pts}, Scalar(0, 0, 255));
        addWeighted(overlay, 0.2, frame, 0.8, 0, frame);
    }

    // === 오프셋 슬라이더 시각화 함수 ===
    // - 화면 상단에 막대 + 점으로 오프셋(px) 표시
    // - 모드(LANE_ONE / LANE_TWO)와 현재 오프셋 값도 텍스트로 출력
    Mat drawOffsetSlider(const Mat& bgr, float offset_px, LaneMode mode) const {
        int sw = frame_width_, sh = 50;
        Mat slider(sh, sw, CV_8UC3, Scalar(50,50,50)); // 회색 배경
        int cx = sw/2; // 중앙 기준선
        line(slider, Point(cx,0), Point(cx,sh-1), Scalar(150,150,150), 1);

        // 오프셋에 따라 빨간 점 표시
        int dot_x = cx + static_cast<int>(std::round(offset_px));
        dot_x = std::clamp(dot_x, 0, sw-1);
        circle(slider, Point(dot_x, sh/2), 6, Scalar(0,0,255), FILLED);

        // 모드와 오프셋 값 텍스트
        string mode_str   = (mode == LaneMode::LANE_ONE) ? "Mode: 1-Lane" : "Mode: 2-Lane";
        string offset_str = "Offset(px): " + std::to_string(static_cast<int>(std::round(offset_px)));
        putText(slider, mode_str,   Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(220,220,220), 1);
        putText(slider, offset_str, Point(10, 42), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(220,220,220), 1);

        // 상단 슬라이더 + 원본 영상 합치기 (세로로 이어붙임)
        Mat out; vconcat(slider, bgr, out);
        return out;
    }

    // === 퍼블리시 + 디버그 ===
    // - offset 값 퍼블리시
    // - 차선 회귀 결과 [m, b] 퍼블리시
    // - 디버그 이미지(dbg_from_fit)가 있으면 그대로 imshow
    // - 항상 오프셋 슬라이더 표시
    void publishAndDebug(const cv::Mat& frame_in,
                        float offset,
                        const LineFit& fit_bev,
                        bool show_dbg,
                        const cv::Mat* dbg_from_fit = nullptr)
    {
        // 작업용 복사본(여기에 중앙선 그려서 보여줌)
        cv::Mat frame = frame_in.clone();

        // 1) BEV 직선을 프레임 좌표의 두 점으로 역투영
        cv::Point2f P0, P1;
        bool mapped = bevLineToFrame(fit_bev, P0, P1);

        // 2) 프레임 기준 x = m*y + b 로 환산 (두 점으로 직선 복원)
        LineFit fit_frame{0.f, 0.f};
        if (mapped) {
            float dy = (P1.y - P0.y);
            if (std::abs(dy) < 1e-6f) {
                // 수평에 가까우면 y 구간 조금 벌려서 안전 처리
                dy = (dy >= 0 ? 1e-6f : -1e-6f);
            }
            fit_frame.m = (P1.x - P0.x) / dy;
            fit_frame.b = P0.x - fit_frame.m * P0.y;

            // // 3) 원본 프레임 위에 중앙선 그리기 (y=0 ~ y=H-1 구간)
            // int H = frame.rows, W = frame.cols;
            // auto clampi = [&](int v, int lo, int hi){ return std::max(lo, std::min(hi, v)); };
            // int x_top = clampi(static_cast<int>(std::round(fit_frame.m * 0.0f      + fit_frame.b)), 0, W-1);
            // int x_bot = clampi(static_cast<int>(std::round(fit_frame.m * (H - 1.0f) + fit_frame.b)), 0, W-1);
            // cv::line(frame, {x_top, 0}, {x_bot, H-1}, cv::Scalar(0,255,0), 2); // 초록 중앙선
        }

        // 4) 오프셋 퍼블리시
        std_msgs::msg::Int16 offset_msg;
        offset_msg.data = static_cast<int16_t>(std::round(offset));
        offset_pub_->publish(offset_msg);

        // 5) 회귀 결과 [m, b] 퍼블리시 — ★ 프레임 기준으로 발행하도록 변경 ★
        std_msgs::msg::Float32MultiArray fit_msg;
        fit_msg.data = { fit_frame.m, fit_frame.b };
        fit_pub_->publish(fit_msg);

        // 6) 디버그: 슬라이딩윈도우 BEV 디버그(있으면) + 프레임 표시
        if (debug_view_ && show_dbg && dbg_from_fit && !dbg_from_fit->empty()) {
            cv::imshow("SlidingWindows", *dbg_from_fit);
        }

        // 4) 오프셋 슬라이더 출력 (항상 표시)
        if (debug_view_ && show_dbg) {
            cv::Mat vis = drawOffsetSlider(frame, offset, lane_mode_);
            cv::imshow("Lane View + Offset", vis);
            cv::waitKey(1);
        }
    }

    // BEV 직선 x = m*y + b 를 BEV 내 두 점으로 잡고, 원본 프레임 좌표로 역투영
    bool bevLineToFrame(const LineFit& lf, cv::Point2f& p_frame0, cv::Point2f& p_frame1) {
        if (bev_size_.width <= 1 || bev_size_.height <= 1 || H_inv_.empty()) return false;

        // BEV 세로 양 끝 점(상단/하단)에서 x 계산
        float y0 = 0.0f;
        float y1 = static_cast<float>(bev_size_.height - 1);
        float x0 = lf.m * y0 + lf.b;
        float x1 = lf.m * y1 + lf.b;

        // BEV 범위 살짝 클램프(안전)
        auto clampf = [](float v, float lo, float hi){ return std::max(lo, std::min(hi, v)); };
        x0 = clampf(x0, 0.0f, static_cast<float>(bev_size_.width  - 1));
        x1 = clampf(x1, 0.0f, static_cast<float>(bev_size_.width  - 1));

        std::vector<cv::Point2f> src = { {x0, y0}, {x1, y1} };
        std::vector<cv::Point2f> dst;
        cv::perspectiveTransform(src, dst, H_inv_); // BEV→프레임

        if (dst.size() != 2) return false;
        p_frame0 = dst[0];
        p_frame1 = dst[1];
        return true;
    }

    // 변경 성공/진행 상태 업데이트 + 퍼블리시
    void updateLaneChangeState(bool valid, const LineFit& center_fit, float offset) {
        const bool mode_changing = (current_mode_ == 5);
        int changing_flag = mode_changing ? 1 : 0;

        if (mode_changing && valid) {
            const float off = std::abs(offset);
            const bool  is_curve    = (std::abs(center_fit.m) >= M_SPLIT_); // ← 여기!
            const float tol_settle  = is_curve ? TOL_CURVE_ : TOL_STRAIGHT_;

            if (change_phase_ == WAIT_SPIKE) {
                if (off >= TOL_CHANGE_) {
                    change_phase_ = WAIT_SETTLE;
                    stable_streak_ = 0;
                }
            } else { // WAIT_SETTLE
                if (off <= tol_settle) stable_streak_++;
                else                   stable_streak_ = 0;

                if (stable_streak_ >= STREAK_NEED_) {
                    success_pulse_ = success_pulse_frames_;
                    change_phase_  = WAIT_SPIKE;
                    stable_streak_ = 0;
                }
            }
        } else {
            change_phase_  = WAIT_SPIKE;
            stable_streak_ = 0;
        }

        std_msgs::msg::Int32MultiArray st;
        st.data = { changing_flag, (success_pulse_ > 0 ? 1 : 0) };
        lane_change_state_pub_->publish(st);
        if (success_pulse_ > 0) success_pulse_--;
    }

    // lane mode → 목표 ref 비율(0~1)
    float getTargetRefForMode(LaneMode m) const {
        float r = (m == LaneMode::LANE_ONE) ? center_reference_lane_one_
                                            : center_reference_lane_two_;
        return std::clamp(r, 0.0f, 1.0f);
    }

    // 현재 사용할 ref 비율(스무딩 ON이면 현재 보간, OFF면 목표값 즉시)
    float getActiveRefRatio() const {
        return smooth_enabled_ ? ref_ratio_current_ : getTargetRefForMode(lane_mode_);
    }

    // 전환 시작: 현재값→목표값을 duration 동안 선형보간
    void startRefTransition(LaneMode new_mode) {
        if (!smooth_enabled_) return;
        ref_ratio_start_   = ref_ratio_current_;
        ref_ratio_target_  = getTargetRefForMode(new_mode);
        ref_start_time_    = this->now();
        ref_transition_active_ = true;
    }

    // 매 프레임 호출해서 ref_ratio_current_를 갱신
    void updateRefRatio() {
        if (!smooth_enabled_ || !ref_transition_active_) return;
        const double t = (this->now() - ref_start_time_).seconds();
        if (t >= ref_transition_duration_sec_) {
            ref_ratio_current_     = ref_ratio_target_;
            ref_transition_active_ = false;
            return;
        }
        const double dur = std::max(1e-6, ref_transition_duration_sec_);
        const float  a   = static_cast<float>(t / dur);      // 0~1
        ref_ratio_current_ = ref_ratio_start_ + (ref_ratio_target_ - ref_ratio_start_) * a;
    }


private:
    // ===== ROS 통신 객체 =====
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mode_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr offset_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fit_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr lane_change_state_pub_;

    // ===== 파라미터 (Config에서 로드) =====
    Config config_; // JSON에서 불러온 설정값
    LaneMode lane_mode_; // 현재 차선 모드 (1차선 / 2차선)
    int frame_width_, frame_height_; // 입력 프레임 크기
    int roi_top_width_, roi_bottom_width_; // ROI 사다리꼴의 상단/하단 폭
    int roi_top_y_, roi_bottom_y_; // ROI 사다리꼴의 상단/하단 y좌표
    float center_reference_lane_one_; // 1차선일 때 기준 비율
    float center_reference_lane_two_; // 2차선일 때 기준 비율

    // ===== BEV 관련 =====
    cv::Mat H_;      // 사다리꼴 → BEV
    cv::Mat H_inv_;  // BEV → 사다리꼴(원본 프레임)
    cv::Size bev_size_; // BEV 영상 크기

    // 직전 프레임의 중앙선/오프셋 기록 (노이즈 발생 시 fallback용)
    LineFit prev_center_fit_{0.f, 0.f};
    bool    has_prev_center_fit_ = false;
    float   prev_offset_ = 0.f;
    int ref_x_ = 0;

    // ===== 디버그 관련 =====
    int frame_count_ = 0; // 프레임 카운터
    int debug_stride_ = 1; // 디버그 출력 주기 (1이면 매 프레임)
    bool debug_view_ = config_.debug_view; // 시각화 여부 (Config에서 불러옴)

    // ===== 차선 변경 상태 추적 =====
    int  current_mode_ = 0;
    int  current_lane_ = 0;
    bool check_active_ = false;      // 5→3 전환 후 "성공 검증" 진행 중

    // ===== 차선 변경 상태 추적 임계값 (필요시 Config로 빼도 됨) =====
    float TOL_STRAIGHT_ = config_.lane_change_tol_straight; // 직선에서 허용 offset
    float TOL_CURVE_    = config_.lane_change_tol_curve; // 곡선에서 허용 offset (직선보다 넓게)
    float TOL_CHANGE_   = config_.lane_change_tol_change; // "차선 변경 순간"으로 볼 offset (둘보다 확 넓게)
    int   STREAK_NEED_  = config_.lane_change_streak_need;    // 안정 프레임 연속 필요 개수
    float M_SPLIT_      = config_.lane_change_m_split; // |m|>=M_SPLIT_이면 곡선으로 간주

    enum Phase { WAIT_SPIKE=0, WAIT_SETTLE=1 } change_phase_ = WAIT_SPIKE;
    int   stable_streak_ = 0;
    int   success_pulse_ = 0;        // 성공 1프레임 펄스
    int   success_pulse_frames_ = 1; // 펄스 길이(원하면 2~3)
    
    // ===== lane ref(센터 비율) 스무딩 전환 =====
    bool        smooth_enabled_ = config_.change_ref_smoothly;     // ← 추가: config 스위치
    float       ref_ratio_current_      = 0.f;   // 지금 쓰는 기준 비율(0~1)
    float       ref_ratio_start_        = 0.f;   // 전환 시작 시점 값
    float       ref_ratio_target_       = 0.f;   // 전환 목표 값
    rclcpp::Time ref_start_time_;                // 전환 시작 시간
    bool        ref_transition_active_  = false; // 전환 중인지
    double      ref_transition_duration_sec_ = 0.8; // 전환 소요 시간(초), config으로 오버라이드

    // ===== 콜백: 카메라 영상 처리 파이프라인 =====
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {

        // 연산 속도 측정 시작
        auto start = std::chrono::steady_clock::now();

        if (smooth_enabled_) updateRefRatio(); // ← 추가

        // (0) ROS 이미지 → OpenCV Mat 변환
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
            return;
        }
        Mat frame = cv_ptr->image;
        if (frame.empty()) return;

        // ROI 사다리꼴 시각화 (디버그용)
        Mat frame_roi_vis = frame.clone();
        drawROIPolygon(frame_roi_vis);
        if (debug_view_) {
            imshow("ROI Polygon", frame_roi_vis);
        }

        // (1) 노란선 전처리 → ROI 내부의 노란 에지 픽셀만 추출
        Mat yellow_edges = preprocessYellow(frame);
        if (debug_view_) {
            imshow("Mask-Yellow", yellow_edges);
        }

        // (2) BEV 변환: 사다리꼴 ROI → 직사각형 투시변환
        Mat bev_yellow;
        warpPerspective(yellow_edges, bev_yellow, H_, bev_size_, INTER_LINEAR, BORDER_CONSTANT,  Scalar(0));
        if (debug_view_) {
            imshow("BEV-Yellow", bev_yellow);
        }

        // (3) 수평 노이즈 행 제거
        const int y_start_chk = (int)std::round(bev_size_.height * 0.0f);              // 예: 상하단 일부 제외하고 검사
        const int y_end_chk   = (int)std::round(bev_size_.height * 1.0f);
        

        // BEV가 이진(CV_8UC1)일 때, 컬러 디버그 캔버스 준비
        cv::Mat bev_color;
        cv::cvtColor(bev_yellow, bev_color, cv::COLOR_GRAY2BGR);

        // corridor 좌/우 경계
        int x_min = ref_x_ - static_cast<int>(config_.corridor_width / 2);
        int x_max = ref_x_ + static_cast<int>(config_.corridor_width / 2);
        

        // 3-1) 수평 줄 억제
        cv::Mat bev_clean;
        bool suppressed = suppressHorizontalNoiseRows(
            bev_yellow, bev_clean,
            config_.horizontal_noise_width,
            config_.horizontal_band_corridor_ratio_thresh,
            y_start_chk, y_end_chk,
            config_.horizontal_noise_band_h,
            config_.horizontal_noise_extra_pad,
            x_min, x_max,
            &bev_color
        );

        // 필요하면 바뀐 BEV 사용
        if (suppressed) {
            bev_yellow = bev_clean; // 이후 파이프라인은 억제된 영상 사용
        }

        // 3-2) 열 합 기반 피크 억제 (추가)
        cv::Mat bev_clean2;
        // corridor만 쓸지 전체폭 쓸지 선택.
        // vertical_noise_peak_use_corridor true면 corridor만 검사, false면 전체 폭
        int cx_min = config_.vertical_noise_peak_use_corridor ? x_min : 0;
        int cx_max = config_.vertical_noise_peak_use_corridor ? x_max : (bev_size_.width - 1);

        bool suppressed2 = suppressColumnBands(
            bev_yellow, bev_clean2,
            cx_min, cx_max,
            y_start_chk, y_end_chk,
            config_.vertical_noise_band_w,     // 새로 추가할 band 폭
            config_.vertical_noise_min_pixels, // 최소 픽셀 기준
            config_.vertical_noise_peak_ratio, // 최대 대비 비율
            config_.vertical_noise_extra_pad_half_width, // 주변 여유 폭
            &bev_color // 빨간 칠
        );
        if (suppressed2) {
            bev_yellow = bev_clean2;
        }

        // 디버그 표시
        if (debug_view_) {
            cv::imshow("BEV-Yellow (suppressed rows/columns in red)", bev_color);
        }

        // 디버그 스로틀 적용
        frame_count_++;
        bool show_dbg = debug_view_ && (frame_count_ % debug_stride_ == 0);

        // (4) BEV에서 슬라이딩 윈도우로 중앙선 추출 + 직선 피팅
        bool valid = false;
        cv::Mat dbg;
        LineFit center_fit = fitLaneFromBEV(bev_yellow, valid, show_dbg ? &dbg : nullptr);

        // (5) 오프셋 계산 및 히스토리 업데이트
        float offset = 0.f;
        if (valid) {
            offset = calcOffsetFromCenterLine(center_fit, bev_size_.width);
            prev_offset_ = offset;               // 직전 offset 갱신
            prev_center_fit_ = center_fit;          // 직전 라인 갱신
            has_prev_center_fit_ = true;
        } else {
            if (has_prev_center_fit_) {
                offset = prev_offset_; // 직전값 재사용
                // RCLCPP_INFO(get_logger(), "Center lane invalid → reuse previous offset: %.1f", offset);
            } else {
                offset = 0.f; // 처음부터 데이터가 없으면 0 사용
                // RCLCPP_INFO(get_logger(), "Center lane invalid and no history → offset=0");
            }
        }

        // offset 계산 발행 + 디버그 출력
        publishAndDebug(frame, offset, center_fit, show_dbg, /*dbg_from_fit=*/(dbg.empty()? nullptr : &dbg));
        
    
        // === 차선 변경 상태 업데이트/퍼블리시 ===
        // updateLaneChangeState(/*valid=*/valid, /*center_fit=*/center_fit, /*offset=*/offset);

        // 연산 속도 측정 끝 //
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        // std::cout << "Cycle time: " << duration.count() << " us" << std::endl;
        // 연산 속도 측정 끝 //
        
        
    }

    // ===== 콜백: 모드/레인 변경 =====
    void modeCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        // 기대 형식: [mode, lane]  (lane=0→1차선, lane=1→2차선)
        if (msg->data.size() < 2) return;

        const int new_mode = msg->data[0];
        const int new_lane = msg->data[1];

        const int prev_mode = current_mode_;
        current_mode_ = new_mode;
        current_lane_ = new_lane;

        // ===================== lane_mode_ 갱신 규칙 =====================
        // 1) 변경 모드(5)일 때는 그대로 갱신
        // 2) "모드 3이 아니었다가 → 3이 된 순간"에도 갱신
        // if (new_mode == 5 || (prev_mode != 3 && new_mode == 3)) {
        //     lane_mode_ = (new_lane == 0) ? LaneMode::LANE_ONE : LaneMode::LANE_TWO;
        // }

        // lane_mode_ 변경 감지 후, 부드러운 전환 시작
        LaneMode old_lane_mode = lane_mode_;

        // 기존 규칙대로 lane_mode_ 갱신
        if (new_mode == 5 || (prev_mode != 3 && new_mode == 3)) {
            lane_mode_ = (new_lane == 0) ? LaneMode::LANE_ONE : LaneMode::LANE_TWO;
        }

        // 바뀌었으면 ref 전환 시작
        if (smooth_enabled_ && lane_mode_ != old_lane_mode) {
            startRefTransition(lane_mode_);
        }

        // ============================================================

        // 1) 5로 "진입"할 때만 검증 시작
        if (prev_mode != 5 && current_mode_ == 5) {
            check_active_ = true;      // 모드 5에서 계속 검증
            stable_streak_ = 0;
            success_pulse_ = 0;
            change_phase_    = WAIT_SPIKE;
        }

        // 2) 5에서 벗어나면(외부가 3으로 바꿔줄 때 등) 상태 리셋
        if (prev_mode == 5 && current_mode_ != 5) {
            check_active_    = false;
            stable_streak_   = 0;
            success_pulse_   = 0;
            change_phase_    = WAIT_SPIKE;
        }
    }
};

int main(int argc, char** argv) {
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
    rclcpp::init(argc, argv);

    // JSON 파라미터 파일 로드 (경로 환경에 맞게 수정)
    Config config = load_config(
        "/home/xytron/xycar_ws/src/orda/modular/lane_detection/lane_detection_parameter.json"
    );

    // 노드 생성 및 실행
    auto node = std::make_shared<LaneDetector>(config);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}