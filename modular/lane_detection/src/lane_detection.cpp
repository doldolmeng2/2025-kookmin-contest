#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <cmath>

#include "lane_detection/config_types.hpp"
#include "lane_detection/config_io.hpp"
#include "lane_detection/preprocess.hpp"
#include "lane_detection/offset_calculate.hpp"

using std::placeholders::_1;

namespace {
inline int16_t clamp_i16(long v) {
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return static_cast<int16_t>(v);
}
}

class LaneDetectionNode : public rclcpp::Node {
public:
    LaneDetectionNode()
    : Node("lane_detection")
    {
        // 기본 파라미터
        this->declare_parameter<std::string>("config_path", "/home/doldolmeng2/xycar_ws/src/orda/modular/lane_detection/lane_config.json");
        this->declare_parameter<bool>("show_debug", false);

        config_path_ = this->get_parameter("config_path").as_string();
        show_debug_  = this->get_parameter("show_debug").as_bool();

        // 전처리용 설정(JSON)
        std::string err; bool loaded = false;
        cfg_ = lane::config::io::LoadOrDefault(config_path_, &loaded, &err);
        if (!loaded) {
            RCLCPP_WARN(get_logger(), "Config load failed, using defaults. %s", err.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Config loaded: %s", config_path_.c_str());
        }

        // (mask_tuner와 동일) 직선/오프셋 검출 파라미터를 ROS 파라미터로 선언/적용
        declare_detect_params();
        update_detect_params_from_node();

        if (show_debug_) {
            cv::namedWindow("LD White",   cv::WINDOW_NORMAL);
            cv::namedWindow("LD Yellow",  cv::WINDOW_NORMAL);
            cv::namedWindow("LD Y-Post",  cv::WINDOW_NORMAL);
            cv::namedWindow("LD Overlay", cv::WINDOW_NORMAL);
            cv::resizeWindow("LD White",   640, 360);
            cv::resizeWindow("LD Yellow",  640, 360);
            cv::resizeWindow("LD Y-Post",  640, 360);
            cv::resizeWindow("LD Overlay", 800, 450);
        }

        // publisher: lane_offset(Int16)
        pub_offset_ = this->create_publisher<std_msgs::msg::Int16>("lane_offset", 10);

        // subscribers
        sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/resized_image", rclcpp::SensorDataQoS(),
            std::bind(&LaneDetectionNode::imageCb, this, _1)
        );
        sub_mode_ = this->create_subscription<std_msgs::msg::String>(
            "mode_info", 10, std::bind(&LaneDetectionNode::modeCb, this, _1)
        );

        RCLCPP_INFO(get_logger(), "lane_detection ready. show_debug=%d", show_debug_);
    }

    ~LaneDetectionNode() override {
        if (show_debug_) cv::destroyAllWindows();
    }

private:
    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr   sub_mode_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       pub_offset_;

    // 전처리 설정(JSON)
    lane::config::LaneConfig cfg_;
    std::string config_path_;
    bool show_debug_ = false;

    // 라인/오프셋 검출 파라미터 (mask_tuner와 동일하게 ROS 파라미터에서 받음)
    lane::offset::DetectParams dp_;

    // offset 유지
    bool    have_last_offset_ = false;
    int16_t last_offset_i16_  = 0;

    // mode_info 저장용(지금은 저장만)
    std::string mode_ = "unknown";
    int lane_mode_ = 1; // 0: 1차선, 1: 2차선

    // ---- mode_info ----
    void modeCb(const std_msgs::msg::String::SharedPtr msg) {
        // 기대 포맷: "mode,lane" (예: "auto,1")
        const std::string s = msg->data;
        auto comma = s.find(',');
        if (comma == std::string::npos) {
            mode_ = s; // lane 값은 유지
        } else {
            mode_ = s.substr(0, comma);
            try { lane_mode_ = std::stoi(s.substr(comma + 1)); } catch (...) {}
        }
        // 지금은 저장만
    }

    // ---- image ----
    void imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat bgr;
        try {
            bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "cv_bridge: %s", e.what());
            return;
        }
        if (bgr.empty()) return;

        // (선택) 런타임에 det.* 파라미터가 바뀌었을 수 있으므로 매 콜백에서 갱신
        update_detect_params_from_node();

        // --- Preprocess (lane_config.json 기반) ---
        // White (Lab -> YCrCb 직렬)
        cv::Mat w_mask, w_vis;
        lane::preprocess::WMaskProcessor(
            bgr, cfg_.white_mask, w_mask,
            cfg_.white_mask.visualize ? &w_vis : nullptr
        );

        // Yellow (HLS -> HSV 직렬)
        cv::Mat y_mask, y_vis;
        lane::preprocess::YMaskProcessor(
            bgr, cfg_.yellow_mask, y_mask,
            cfg_.yellow_mask.visualize ? &y_vis : nullptr
        );

        // Yellow Post (Open/Close + Sobel X)
        cv::Mat y_post;
        lane::preprocess::YImageProcessor(y_mask, cfg_.yellow_image, y_post);

        // --- 오프셋 계산 (offset_calculate.* 사용) ---
        cv::Mat overlay = bgr.clone();

        // 흰 라인
        lane::offset::LineFit wline = lane::offset::WLineCalculate(w_mask, overlay, dp_);

        // 노란 라인(후처리 결과 우선)
        const cv::Mat& y_input = (y_post.empty() ? y_mask : y_post);
        lane::offset::OffsetViz ov{};
        lane::offset::LineFit yline = lane::offset::YLineCalculate(y_input, overlay, dp_, &wline, &ov);

        // --- lane_offset 퍼블리시(Int16) ---
        std_msgs::msg::Int16 out;
        if (ov.valid) {
            int16_t v = clamp_i16(std::lround(ov.offset_px));
            out.data = v;
            last_offset_i16_ = v;
            have_last_offset_ = true;
        } else if (have_last_offset_) {
            out.data = last_offset_i16_;
        } else {
            out.data = 0; // 초기값
        }
        pub_offset_->publish(out);

        // --- 디버그 표시 ---
        if (show_debug_) {
            if (!w_vis.empty()) cv::imshow("LD White", w_vis);
            else                cv::imshow("LD White", bgr);

            if (!y_vis.empty()) cv::imshow("LD Yellow", y_vis);
            else                cv::imshow("LD Yellow", bgr);

            if (!y_post.empty()) cv::imshow("LD Y-Post", y_post);

            cv::imshow("LD Overlay", overlay);
            (void)cv::waitKey(1);
        }
    }

    // ---- DetectParams: mask_tuner와 동일 ----
    void declare_detect_params() {
        this->declare_parameter<int>("det.canny_low", 0);
        this->declare_parameter<int>("det.canny_high", 0);
        this->declare_parameter<int>("det.canny_aperture", 3);

        this->declare_parameter<double>("det.rho", 1.0);
        this->declare_parameter<double>("det.theta_deg", 1.0); // degrees
        this->declare_parameter<int>("det.hough_thresh", 30);
        this->declare_parameter<double>("det.min_line_length", 60.0);
        this->declare_parameter<double>("det.max_line_gap", 10.0);

        // 화이트: 우하단 시작 ROI
        this->declare_parameter<double>("det.start_x_min_frac", 0.55);
        this->declare_parameter<double>("det.start_x_max_frac", 0.98);
        this->declare_parameter<double>("det.start_y_min_frac", 0.60);
        this->declare_parameter<double>("det.start_y_max_frac", 0.98);

        // 노란 라인 보정(브리징/각도/유지 프레임 등)
        this->declare_parameter<int>("det.yellow_bridge_kernel", 5);
        this->declare_parameter<double>("det.y_angle_min_deg", 15.0);
        this->declare_parameter<double>("det.y_angle_max_deg", 88.0);
        this->declare_parameter<int>("det.hold_frames", 8);
    }

    void update_detect_params_from_node() {
        dp_.canny_low  = this->get_parameter("det.canny_low").as_int();
        dp_.canny_high = this->get_parameter("det.canny_high").as_int();
        dp_.canny_aperture = this->get_parameter("det.canny_aperture").as_int();

        dp_.rho   = this->get_parameter("det.rho").as_double();
        double theta_deg = this->get_parameter("det.theta_deg").as_double();
        dp_.theta = theta_deg * CV_PI / 180.0;
        dp_.hough_thresh    = this->get_parameter("det.hough_thresh").as_int();
        dp_.min_line_length = this->get_parameter("det.min_line_length").as_double();
        dp_.max_line_gap    = this->get_parameter("det.max_line_gap").as_double();

        dp_.start_x_min_frac = this->get_parameter("det.start_x_min_frac").as_double();
        dp_.start_x_max_frac = this->get_parameter("det.start_x_max_frac").as_double();
        dp_.start_y_min_frac = this->get_parameter("det.start_y_min_frac").as_double();
        dp_.start_y_max_frac = this->get_parameter("det.start_y_max_frac").as_double();

        dp_.yellow_bridge_kernel = this->get_parameter("det.yellow_bridge_kernel").as_int();
        dp_.y_angle_min_deg = this->get_parameter("det.y_angle_min_deg").as_double();
        dp_.y_angle_max_deg = this->get_parameter("det.y_angle_max_deg").as_double();
        dp_.hold_frames = this->get_parameter("det.hold_frames").as_int();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
