#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "parameter_loader.hpp"
#include <opencv2/core/version.hpp>

using namespace std;
using namespace cv;

struct LaneLines {
    Vec2f left_fit;
    Vec2f right_fit;
    Vec2f center_fit;
};

class LaneDetector : public rclcpp::Node {
public:
    LaneDetector(const Config& config) 
    : Node("lane_detector_node"), config_(config),
    lane_mode_(config_.lane_mode),
    frame_width_(config_.frame_width), // 필요 시 config에서 불러올 수도 있음
    frame_height_(config_.frame_height),
    roi_height_(static_cast<int>(frame_height_ * config_.roi_height_coefficient)),
    roi_top_width_(static_cast<int>(frame_width_ * config_.roi_top_width_coefficient)),
    roi_bottom_width_(static_cast<int>(frame_width_ * config_.roi_bottom_width_coefficient)),
    center_reference_lane_one_(config_.center_reference_lane_one),
    center_reference_lane_two_(config_.center_reference_lane_two)
    {
        RCLCPP_INFO(this->get_logger(), "roi_bottom_width_: %d", roi_bottom_width_);
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/resized_image", 10,
            std::bind(&LaneDetector::imageCallback, this, std::placeholders::_1)
        );
        mode_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/mode_info", 10,
            std::bind(&LaneDetector::modeCallback, this, std::placeholders::_1)
        );
        offset_pub_ = this->create_publisher<std_msgs::msg::Int16>("/lane_offset", 10);
    }

    Mat applyTrapezoidROI(const Mat& frame, int top_width, int bottom_width, int height) {
        Mat mask = Mat::zeros(frame.size(), CV_8UC1);

        int center_x = frame_width_ / 2;
        int bottom_y = frame_height_;
        int top_y = frame_height_ - height;

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
        int roi_y_start = frame_height_ - roi_height_;
        int roi_y_end = frame_height_;

        // 히스토그램 기반 시작점
        Mat roi_mask = binary_mask(Rect(0, roi_y_start, frame_width_, roi_height_));
        std::vector<int> histogram(frame_width_, 0);
        for (int x = 0; x < frame_width_; ++x) {
            histogram[x] = countNonZero(roi_mask.col(x));
        }
        
        // 2. 시작점 계산
        int base_x;
        if (mask_type == "white") {
            int midpoint = frame_width_ / 2;
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
        int num_windows = config_.sliding_window_num_windows;  // 블록 개수
        int margin = config_.sliding_window_margin; // 탐색 범위(블록 width)
        const size_t minpix = config_.sliding_window_minpix; // 유효한 픽셀이 minpix 이상이면 중심 좌표 업데이트
        int window_height = roi_height_ / num_windows;

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
                    if (x >= 0 && x < frame_width_ && binary_mask.at<uchar>(y, x) > 0) {
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

    LaneLines detectLanes(const Mat& white_mask, const Mat& yellow_mask, const Mat& resized_img) {
        Mat debug_img = resized_img.clone();
        // 흰색 왼쪽 차선
        auto white_left_points = detectLaneFromMask(white_mask, "white", true, debug_img);

        // 흰색 오른쪽 차선
        auto white_right_points = detectLaneFromMask(white_mask, "white", false, debug_img);

        // 노란 중앙선
        auto yellow_points = detectLaneFromMask(yellow_mask, "yellow", false, debug_img);

        // 3. 피팅
        Vec2f left_fit = fitLineLinear(white_left_points);
        Vec2f right_fit = fitLineLinear(white_right_points);
        Vec2f center_fit = fitLineLinear(yellow_points);

        imshow("Sliding Windows", debug_img);
        waitKey(1);

        return { left_fit, right_fit, center_fit };
    }

    Vec2f fitLineLinear(const std::vector<Point>& points) {
        if (points.size() < 2) return Vec2f(0, 0); // 최소 2점 필요

        Mat X(points.size(), 2, CV_32F);
        Mat Y(points.size(), 1, CV_32F);

        for (size_t i = 0; i < points.size(); ++i) {
            float y = static_cast<float>(points[i].y);
            X.at<float>(i, 0) = y;
            X.at<float>(i, 1) = 1;
            Y.at<float>(i, 0) = static_cast<float>(points[i].x);
        }

        Mat coeffs;
        solve(X, Y, coeffs, DECOMP_SVD);  // [m, b] 반환

        return Vec2f(coeffs.at<float>(0), coeffs.at<float>(1)); // m, b
    }

    float calculateOffsetByIntersection(const Vec2f& line1, const Vec2f& line2) {
        float m1 = line1[0], b1 = line1[1];
        float m2 = line2[0], b2 = line2[1];

        if (std::abs(m1 - m2) < 1e-5f) {
            // 기울기가 같아서 교차하지 않음 (평행)
            return 0.0f;
        }

        float y = (b2 - b1) / (m1 - m2);
        float x = m1 * y + b1;

        float image_center_x = static_cast<float>(frame_width_) / 2.0f;
        return x - image_center_x;
    }

    float calculateOffsetByCenterLane(const cv::Vec2f& line1, LaneMode lane_mode) {
        float slope = line1[0];
        float intercept = line1[1];

        // y 좌표 기준 
        float y1 = frame_height_ * 0.5f;
        float y2 = frame_height_ * 0.8f;
        // float y3 = frame_height_ * 0.75f;

        // 각각의 y값에서 x 좌표를 계산 (x = m*y + b)
        float x1 = slope * y1 + intercept;
        float x2 = slope * y2 + intercept;
        // float x3 = (y3 - intercept) / slope;

        float x_sum = x1 + x2;

        // 기준값 설정 (lane_mode에 따라 다르게), 중앙선이 있어야 할 자리를 정해둠
        float reference = 0.0f;
        if (lane_mode == LaneMode::LANE_ONE) {
            reference = frame_width_ * center_reference_lane_one_;
        } else if (lane_mode == LaneMode::LANE_TWO) {
            reference = frame_width_ * center_reference_lane_two_;
        }

        // 참고: 중앙선이 오른쪽으로 가게 하려면 내가 왼쪽으로 이동해야 함.
        float offset = x_sum - reference;

        return offset;
    }

    void drawLaneLine(Mat& img, const Vec2f& coeffs, const Scalar& color) {
        float m = coeffs[0];
        float b = coeffs[1];
        Point pt1, pt2;

        pt1.y = 0;
        pt1.x = static_cast<int>(m * pt1.y + b);

        pt2.y = img.rows;
        pt2.x = static_cast<int>(m * pt2.y + b);

        line(img, pt1, pt2, color, 2);
    }

    std::pair<Mat, Mat> preprocessImage(const Mat& frame) {
        Mat roi_frame = applyTrapezoidROI(frame, roi_top_width_, roi_bottom_width_, roi_height_);
        Mat blurred, hsv;
        GaussianBlur(roi_frame, blurred, Size(config_.gaussian_blur_kernel_size, config_.gaussian_blur_kernel_size), 0);
        cvtColor(roi_frame, hsv, COLOR_BGR2HSV);

        // --- Yellow 처리: 그대로 유지 ---
        Mat yellow_mask, thick_edges_yellow, edges_yellow;
        inRange(hsv,
                Scalar(config_.yellow_min_h, config_.yellow_min_s, config_.yellow_min_v),
                Scalar(config_.yellow_max_h, config_.yellow_max_s, config_.yellow_max_v),
                yellow_mask);

        Mat kernel_yellow_closing = getStructuringElement(MORPH_RECT, Size(config_.kernel_yellow_closing_size, config_.kernel_yellow_closing_size));
        morphologyEx(yellow_mask, thick_edges_yellow, MORPH_CLOSE, kernel_yellow_closing);

        Mat kernel_yellow_opening = getStructuringElement(MORPH_RECT, Size(config_.kernel_yellow_opening_size, config_.kernel_yellow_opening_size));
        morphologyEx(thick_edges_yellow, thick_edges_yellow, MORPH_OPEN, kernel_yellow_opening);

        cv::Canny(thick_edges_yellow, edges_yellow, config_.canny_yellow_low_threshold, config_.canny_yellow_high_threshold);

        // --- White 처리: Blur→Canny / HSV→Mask → and ---
        Mat edges_white_raw, thick_edges_white, white_mask, edges_white_final;

        // 1. Blur → Canny
        cv::Canny(blurred, edges_white_raw, config_.canny_white_low_threshold, config_.canny_white_high_threshold);

        // 2. Morphology (closing + opening)
        Mat kernel_white_closing = getStructuringElement(MORPH_RECT, Size(config_.kernel_white_closing_size, config_.kernel_white_closing_size));
        morphologyEx(edges_white_raw, thick_edges_white, MORPH_CLOSE, kernel_white_closing);

        Mat kernel_white_opening = getStructuringElement(MORPH_RECT, Size(config_.kernel_white_opening_size, config_.kernel_white_opening_size));
        morphologyEx(thick_edges_white, thick_edges_white, MORPH_OPEN, kernel_white_opening);

        // 3. HSV → white mask
        inRange(hsv,
                Scalar(config_.white_min_h, config_.white_min_s, config_.white_min_v),
                Scalar(config_.white_max_h, config_.white_max_s, config_.white_max_v),
                white_mask);

        // 4. 최종 white edge = and(thick_edges_white, white_mask)
        cv::bitwise_and(thick_edges_white, white_mask, edges_white_final);

        // 디버깅용 출력 (필요 시)
        imshow("White Edge Final", edges_white_final);
        imshow("Yellow Edge Final", edges_yellow);
        waitKey(1);

        return { edges_white_final, edges_yellow };
    }

    void drawLaneOffsetSlider(const cv::Mat& resized_img, int frame_width_, float offset, LaneMode lane_mode_) {
        // 슬라이더 이미지 생성 (길이: frame_width_, 높이: 50)
        int slider_width = frame_width_;
        int slider_height = 50;
        Mat slider(slider_height, slider_width, CV_8UC3, Scalar(50, 50, 50));

        // 중앙선 그리기
        line(slider, Point(slider_width/2, 0), Point(slider_width/2, slider_height), Scalar(150, 150, 150), 1);

        // offset 시각화용 점 그리기
        int center_x = slider_width / 2;
        int dot_x = static_cast<int>(center_x + offset);  // offset을 그대로 픽셀로 사용
        dot_x = std::max(0, std::min(slider_width - 1, dot_x));  // 이미지 경계 안으로 클램프
        circle(slider, Point(dot_x, slider_height/2), 6, Scalar(0, 0, 255), -1);

        // 🟡 텍스트 추가: 현재 차선 모드 + offset 값
        std::string mode_str = (lane_mode_ == LaneMode::LANE_ONE) ? "Mode: 1-Lane" : "Mode: 2-Lane";
        std::string offset_str = "Offset: " + std::to_string(offset);
        
        // 텍스트 출력 위치 (왼쪽 위에 나란히 표시)
        int font_face = cv::FONT_HERSHEY_SIMPLEX;
        double font_scale = 0.6;
        int thickness = 1;

        putText(slider, mode_str, Point(10, 20), font_face, font_scale, Scalar(200, 200, 200), thickness);
        putText(slider, offset_str, Point(10, 40), font_face, font_scale, Scalar(200, 200, 200), thickness);

        // 디버그 출력
        // 위쪽에 offset 슬라이더를 그리고 아래에 영상 보여주기
        Mat combined;
        vconcat(slider, resized_img, combined);  // slider가 위, resized_img가 아래
        imshow("Lane View + Offset", combined);
        waitKey(1);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mode_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr offset_pub_;
    rclcpp::TimerBase::SharedPtr lane_change_timer_;
    Config config_;
    LaneMode lane_mode_;
    int frame_width_;
    int frame_height_;
    int roi_height_;
    int roi_top_width_;
    int roi_bottom_width_;
    bool lane_change_;
    float center_reference_lane_one_;
    float center_reference_lane_two_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // ROS 이미지 → OpenCV Mat
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            Mat resized_img = cv_ptr->image;

            // 전처리
            auto [white_mask, yellow_mask] = preprocessImage(resized_img);

            // 차선 검출
            // 주의! x = m·y + b을 모델링하고(y=mx+b가 아님!!), m,b를 넘겨주는 것임.
            // 차선이나 객체의 수직 형태를 모델링할 때 적합한 방식이라서 사용함.
            LaneLines lines = detectLanes(white_mask, yellow_mask, resized_img);

            // 차선 곡선 그리기
            drawLaneLine(resized_img, lines.left_fit, Scalar(255, 0, 0));    // 왼쪽 차선: 파랑
            drawLaneLine(resized_img, lines.center_fit, Scalar(0, 255, 0));  // 중앙 노란선: 초록
            drawLaneLine(resized_img, lines.right_fit, Scalar(0, 0, 255));   // 오른쪽 차선: 빨강

            float offset;
            if (lane_change_){  // for test: lane_change_ -> true
                offset = calculateOffsetByCenterLane(lines.center_fit, lane_mode_);
            } else {
                if (lane_mode_ == LaneMode::LANE_ONE) {
                    offset = calculateOffsetByIntersection(lines.left_fit, lines.center_fit);
                } else {
                    offset = calculateOffsetByIntersection(lines.center_fit, lines.right_fit);
                }
            }

            if (offset > 400){
                offset = 400;
            } else if (offset < -400){
                offset = -400;
            }

            // offset 퍼블리시
            std_msgs::msg::Int16 offset_msg;
            offset_msg.data = static_cast<int16_t>(offset);
            offset_pub_->publish(offset_msg);
    
            // 조향각 시각화
            drawLaneOffsetSlider(resized_img, frame_width_, offset, lane_mode_);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 예외: %s", e.what());
            return;
        }
    }

    void modeCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data[0] == 3){ // data는 [mode, lane] 형식임. mode 3: 차선주행, mode 4: 장애물 접근, mode 5: 차선 변경 모드, lane 0: 1차선, mode 1: 2차선    
            lane_change_ = false;
        } else if (msg->data[0] == 5){ // data는 [mode, lane] 형식임. mode 3: 차선주행, mode 4: 장애물 접근, mode 5: 차선 변경 모드, lane 0: 1차선, mode 1: 2차선
            if (msg->data[1] == 0){
                lane_mode_ = LaneMode::LANE_ONE;
            } else {
                lane_mode_ = LaneMode::LANE_TWO;
            }
            lane_change_ = true;
        }
    }
};

int main(int argc, char** argv) {
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
    rclcpp::init(argc, argv);

    // json 경로는 개발 환경에 맞게 변경하시면 됩니다.
    Config config = load_config("/home/xytron/xycar_ws/src/orda/modular/lane_detection/lane_detection_parameter.json"); 

    auto node = std::make_shared<LaneDetector>(config);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

