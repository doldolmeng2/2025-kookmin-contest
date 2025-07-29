#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

struct LaneCurves {
    Vec3f left_fit;
    Vec3f right_fit;
    Vec3f center_fit;
};

enum class LaneMode {
    ONE_LANE,  // 왼쪽 흰색 + 중앙 노란선 기준
    TWO_LANE   // 중앙 노란선 + 오른쪽 흰색 기준
};

constexpr int FRAME_WIDTH = 640;
constexpr int FRAME_HEIGHT = 360;

constexpr int roi_height = static_cast<int>(FRAME_HEIGHT * 0.6);
constexpr int roi_top_width = static_cast<int>(FRAME_WIDTH * 0.6);
constexpr int roi_bottom_width = static_cast<int>(FRAME_WIDTH * 3);


class LaneDetector : public rclcpp::Node {
public:
    LaneDetector() : Node("lane_detector_node") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&LaneDetector::imageCallback, this, std::placeholders::_1)
        );
    }

    Mat applyTrapezoidROI(const Mat& frame, int top_width, int bottom_width, int height) {
        Mat mask = Mat::zeros(frame.size(), CV_8UC1);

        int img_width = frame.cols;
        int img_height = frame.rows;

        int center_x = img_width / 2;
        int bottom_y = img_height;
        int top_y = img_height - height;

        // 꼭짓점 계산
        Point pts[1][4];
        pts[0][0] = Point(center_x - top_width / 2, top_y);       // 좌상
        pts[0][1] = Point(center_x + top_width / 2, top_y);       // 우상
        pts[0][2] = Point(center_x + bottom_width / 2, bottom_y); // 우하
        pts[0][3] = Point(center_x - bottom_width / 2, bottom_y); // 좌하

        const Point* ppt[1] = { pts[0] };
        int npt[] = { 4 };

        // 흰색 채우기
        fillPoly(mask, ppt, npt, 1, Scalar(255));

        // ROI 적용 (흰 영역만 남기기)
        Mat roi_applied;
        frame.copyTo(roi_applied, mask);

        return roi_applied;
    }

    std::vector<Point> detectLaneFromMask(const Mat& binary_mask, const std::string& mask_type, bool is_left, Mat& debug_window_vis) {
        int roi_y_start = FRAME_HEIGHT - roi_height;
        int roi_y_end = FRAME_HEIGHT;

        // 히스토그램 기반 시작점
        Mat roi_mask = binary_mask(Rect(0, roi_y_start, FRAME_WIDTH, roi_height));
        std::vector<int> histogram(FRAME_WIDTH, 0);
        for (int x = 0; x < FRAME_WIDTH; ++x) {
            histogram[x] = countNonZero(roi_mask.col(x));
        }
        
        // 2. 시작점 계산
        int base_x;
        if (mask_type == "white") {
            int midpoint = FRAME_WIDTH / 2;
            base_x = is_left
                ? std::distance(histogram.begin(), std::max_element(histogram.begin(), histogram.begin() + midpoint))
                : std::distance(histogram.begin(), std::max_element(histogram.begin() + midpoint, histogram.end()));
        } else if (mask_type == "yellow") {
            base_x = std::distance(histogram.begin(), std::max_element(histogram.begin(), histogram.end()));
        } else {
            std::cerr << "Invalid mask_type: " << mask_type << std::endl;
            return {};
        }

        // Sliding window 파라미터
        int num_windows = 30;  // 블록 개수
        int margin = 70; // 탐색 범위(블록 width)
        const size_t minpix = 5; // 유효한 픽셀이 minpix 이상이면 중심 좌표 업데이트
        int window_height = roi_height / num_windows;

        std::vector<Point> lane_points;
        int x_current = base_x;

        for (int win = 0; win < num_windows; ++win) {
            int win_y_low = roi_y_end - (win + 1) * window_height;
            int win_y_high = roi_y_end - win * window_height;
            int win_x_low = x_current - margin;
            int win_x_high = x_current + margin;

            // 시각화용 사각형 그리기
            rectangle(debug_window_vis, Point(win_x_low, win_y_low), Point(win_x_high, win_y_high),
                    Scalar(0, 255, 255), 1);  // 노란색 선

            std::vector<int> nonzero_x;
            for (int y = win_y_low; y < win_y_high; ++y) {
                for (int x = win_x_low; x < win_x_high; ++x) {
                    if (x >= 0 && x < FRAME_WIDTH && binary_mask.at<uchar>(y, x) > 0) {
                        lane_points.emplace_back(x, y);
                        nonzero_x.push_back(x);
                    }
                }
            }

            if (nonzero_x.size() > minpix) {
                int sum_x = std::accumulate(nonzero_x.begin(), nonzero_x.end(), 0);
                x_current = sum_x / static_cast<int>(nonzero_x.size());
            }
        }

        return lane_points;
    }


    LaneCurves detectLanes(const Mat& white_mask, const Mat& yellow_mask, const Mat& resized_img) {
        Mat debug_img = resized_img.clone();
        // 흰색 왼쪽 차선
        auto white_left_points = detectLaneFromMask(white_mask, "white", true, debug_img);

        // 흰색 오른쪽 차선
        auto white_right_points = detectLaneFromMask(white_mask, "white", false, debug_img);

        // 노란 중앙선
        auto yellow_points = detectLaneFromMask(yellow_mask, "yellow", false, debug_img);

        // 3. 피팅
        Vec3f left_fit = fitPolynomial(white_left_points);
        Vec3f right_fit = fitPolynomial(white_right_points);
        Vec3f center_fit = fitPolynomial(yellow_points);

        imshow("Sliding Windows", debug_img);
        waitKey(1);

        return { left_fit, right_fit, center_fit };
    }

    // 2차 다항식 피팅 (ax² + bx + c)
    Vec3f fitPolynomial(const vector<Point>& points) {
        if (points.size() < 3) return Vec3f(0, 0, 0); // 충분한 점이 없으면 기본값

        Mat X(points.size(), 3, CV_32F);
        Mat Y(points.size(), 1, CV_32F);

        for (size_t i = 0; i < points.size(); ++i) {
            float y = static_cast<float>(points[i].y);
            X.at<float>(i, 0) = y * y;
            X.at<float>(i, 1) = y;
            X.at<float>(i, 2) = 1;
            Y.at<float>(i, 0) = static_cast<float>(points[i].x);
        }

        Mat coeffs;
        solve(X, Y, coeffs, DECOMP_SVD);

        return Vec3f(coeffs.at<float>(0), coeffs.at<float>(1), coeffs.at<float>(2));
    }

    float calculateOffset(const LaneCurves& curves, int img_width, int img_height, LaneMode mode) {
        // y값을 맨 아래 (카메라에 가까운 위치) 기준으로 계산
        float y_eval = static_cast<float>(img_height - 1);

        // 각 차선의 x좌표 계산: x = a*y^2 + b*y + c
        auto eval = [&](const Vec3f& coeffs) {
            return coeffs[0]*y_eval*y_eval + coeffs[1]*y_eval + coeffs[2];
        };

        float left_x = eval(curves.left_fit);
        float right_x = eval(curves.right_fit);
        float center_x = eval(curves.center_fit);  // 중간 노란선

        float lane_center_x;

        if (mode == LaneMode::ONE_LANE) {
            lane_center_x = (left_x + center_x) / 2.0f;
        } else {
            lane_center_x = (center_x + right_x) / 2.0f;
        }

        // 이미지 중앙과의 차이
        float image_center_x = static_cast<float>(img_width) / 2.0f;
        float offset = lane_center_x - image_center_x;

        // 차선 곡률도 고려해서 강화된 offset 계산 (선택사항)
        float curvature_factor = std::abs(curves.center_fit[0]);  // 곡률 계수 a값 (곡선일수록 큼)
        float adjusted_offset = offset * (1.0f + 5.0f * curvature_factor);  // 조정 인자 5.0은 튜닝 가능

        return adjusted_offset;  // 최종 반환
    }
    
    void drawLaneCurve(Mat& img, const Vec3f& coeffs, const Scalar& color) {
        std::vector<Point> curve_points;
        for (int y = 0; y < img.rows; ++y) {
            float x = coeffs[0]*y*y + coeffs[1]*y + coeffs[2];
            if (x >= 0 && x < img.cols) {
                curve_points.emplace_back(static_cast<int>(x), y);
            }
        }
        for (size_t i = 1; i < curve_points.size(); ++i) {
            line(img, curve_points[i - 1], curve_points[i], color, 2);
        }
    }


private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // ROS 이미지 → OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 예외: %s", e.what());
            return;
        }

        // 리사이즈
        Mat resized_img;
        resize(cv_ptr->image, resized_img, Size(FRAME_WIDTH, FRAME_HEIGHT));

        // 전처리
        auto [white_mask, yellow_mask] = preprocessImage(resized_img);

        // 차선 검출
        LaneCurves curves = detectLanes(white_mask, yellow_mask, resized_img);

        // 차선 곡선 그리기
        drawLaneCurve(resized_img, curves.left_fit, Scalar(255, 0, 0));    // 왼쪽 차선: 파랑
        drawLaneCurve(resized_img, curves.center_fit, Scalar(0, 255, 0));  // 중앙 노란선: 초록
        drawLaneCurve(resized_img, curves.right_fit, Scalar(0, 0, 255));   // 오른쪽 차선: 빨강

        // offset 계산 (2차선 기준으로 예시)
        float offset = calculateOffset(curves, resized_img.cols, resized_img.rows, LaneMode::TWO_LANE);

        // 슬라이더 이미지 생성 (길이: FRAME_WIDTH, 높이: 50)
        int slider_width = FRAME_WIDTH;
        int slider_height = 50;
        Mat slider(slider_height, slider_width, CV_8UC3, Scalar(50, 50, 50));

        // 중앙선 그리기
        line(slider, Point(slider_width/2, 0), Point(slider_width/2, slider_height), Scalar(150, 150, 150), 1);

        // offset 시각화용 점 그리기
        int center_x = slider_width / 2;
        int dot_x = static_cast<int>(center_x + offset);  // offset을 그대로 픽셀로 사용
        dot_x = std::max(0, std::min(slider_width - 1, dot_x));  // 이미지 경계 안으로 클램프

        circle(slider, Point(dot_x, slider_height/2), 6, Scalar(0, 0, 255), -1);

        // 디버그 출력
        // 위쪽에 offset 슬라이더를 그리고 아래에 영상 보여주기
        Mat combined;
        vconcat(slider, resized_img, combined);  // slider가 위, resized_img가 아래
        imshow("Lane View + Offset", combined);
        waitKey(1);
    }

    std::pair<Mat, Mat> preprocessImage(const Mat& frame) {
        Mat roi_frame = applyTrapezoidROI(frame, roi_top_width, roi_bottom_width, roi_height);
        // imshow("roi_frame", roi_frame);
        // waitKey(1);
        Mat blurred, hsv, white_mask, yellow_mask, edge_mask, final_mask;

        // 1. Blur로 빛 번짐 제거
        GaussianBlur(roi_frame, blurred, Size(5, 5), 0);
        // imshow("Blurred", blurred);
        // waitKey(1);

        // 2. HSV 변환
        cvtColor(roi_frame, hsv, COLOR_BGR2HSV);
        // imshow("HSV", hsv);
        // waitKey(1);

        // 3. 흰색/노란색 마스킹 (HSV 범위 조정 필요)
        inRange(hsv, Scalar(20, 70, 180), Scalar(50, 255, 255), yellow_mask);
        inRange(hsv, Scalar(50, 0, 180), Scalar(140, 60, 255), white_mask);
        // imshow("Yellow Mask (before Edge)", yellow_mask);
        // waitKey(1); 
        // imshow("White Mask (before Edge)", white_mask);
        // waitKey(1); 

        // 4. Canny Edge 검출
        Mat edges;
        Canny(blurred, edges, 150, 200); // , , 하한선, 상한선
        // imshow("Canny Edges", edges);
        // waitKey(1); 

        // 두껍게 만들기 (팽창 연산)
        Mat thick_edges_yellow;
        Mat kernel_yellow_closing = getStructuringElement(MORPH_RECT, Size(35, 35));
        morphologyEx(edges, thick_edges_yellow, MORPH_CLOSE, kernel_yellow_closing);
        // 2. Opening (노이즈 제거)
        Mat kernel_yellow_opening = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(thick_edges_yellow, thick_edges_yellow, MORPH_OPEN, kernel_yellow_opening);

        Mat thick_edges_white;
        // 1. Closing (점선 연결)
        Mat kernel_white_closing = getStructuringElement(MORPH_RECT, Size(9, 9));
        morphologyEx(edges, thick_edges_white, MORPH_CLOSE, kernel_white_closing);
        // 2. Opening (노이즈 제거)
        Mat kernel_white_opening = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(thick_edges_white, thick_edges_white, MORPH_OPEN, kernel_white_opening);

        // 5. Edge + 마스크 AND 연산
        bitwise_and(white_mask, thick_edges_white, white_mask);
        bitwise_and(yellow_mask, thick_edges_yellow, yellow_mask);
        imshow("Yellow Mask (after Edge)", yellow_mask);
        waitKey(1); 
        imshow("White Mask (after Edge)", white_mask);
        waitKey(1); 

        return { white_mask, yellow_mask };
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

