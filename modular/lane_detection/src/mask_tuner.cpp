#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <string>
#include <cmath>  // for std::round

#include "lane_detection/config_types.hpp"
#include "lane_detection/config_io.hpp"
#include "lane_detection/preprocess.hpp"
#include "lane_detection/offset_calculate.hpp"  // ✅ 추가

using std::placeholders::_1;

namespace {

inline int clampi(int v, int lo, int hi) { return std::max(lo, std::min(v, hi)); }
inline int odd_or_next(int k) { return (k % 2 == 1) ? k : (k + 1); }

// Sobel scale/delta 트랙바를 정수 <-> 실수로 매핑
static constexpr int SCALE_SLIDER_MAX = 50;   // 0.0 ~ 5.0 (step 0.1)
static constexpr int DELTA_SLIDER_MAX = 50;   // 0 ~ 50
inline int  to_scale_slider(double v) { return clampi(static_cast<int>(std::round(v * 10.0)), 0, SCALE_SLIDER_MAX); }
inline double from_scale_slider(int s) { return static_cast<double>(clampi(s, 0, SCALE_SLIDER_MAX)) / 10.0; }
inline int  to_delta_slider(double v) { return clampi(static_cast<int>(std::round(v)), 0, DELTA_SLIDER_MAX); }
inline double from_delta_slider(int s){ return static_cast<double>(clampi(s, 0, DELTA_SLIDER_MAX)); }

// 트랙바/표시 윈도우
const std::string WIN_WHITE  = "White Tuner";
const std::string WIN_YELLOW = "Yellow Tuner";
const std::string WIN_POST   = "Yellow Post";
const std::string WIN_VIZ    = "Offset Viz";   // ✅ 추가

} // namespace

class MaskTunerNode : public rclcpp::Node {
public:
    MaskTunerNode()
    : Node("mask_tuner")
    {
        // 파라미터: config 경로
        this->declare_parameter<std::string>("config_path", "/home/doldolmeng2/xycar_ws/src/orda/modular/lane_detection/lane_config.json");
        config_path_ = this->get_parameter("config_path").as_string();

        // (옵션) 오프셋 검출 파라미터 선언
        declare_detect_params();

        // 설정 로드
        std::string err; bool loaded = false;
        cfg_ = lane::config::io::LoadOrDefault(config_path_, &loaded, &err);
        if (!loaded) {
            RCLCPP_WARN(this->get_logger(), "Config load failed, using defaults. %s", err.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Config loaded from %s", config_path_.c_str());
        }

        // 윈도우 & 트랙바 생성 (안전한 순서)
        create_windows_and_trackbars();

        // 구독자
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/resized_image", rclcpp::SensorDataQoS(),
            std::bind(&MaskTunerNode::image_cb, this, _1)
        );

        // 30Hz 타이머: 최신 프레임 처리 + 키 처리
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&MaskTunerNode::on_timer, this)
        );
    }

private:
    // ==== ROS ====
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ==== 상태 ====
    std::mutex mtx_;
    cv::Mat last_bgr_;
    lane::config::LaneConfig cfg_;
    std::string config_path_;

    // ==== 오프셋 검출 파라미터 ====
    lane::offset::DetectParams dp_;   // ✅

    // ==== 트랙바 값 (HighGUI는 int만 지원) ====

    // White Mask: Lab + YCrCb
    int w_roi_bottom_ = 10, w_roi_top_ = 40;
    int w_lab_l_min_=0,   w_lab_l_max_=255;
    int w_lab_a_min_=110, w_lab_a_max_=150;
    int w_lab_b_min_=140, w_lab_b_max_=200;
    int w_y_min_=0, w_y_max_=255;
    int w_cr_min_=135, w_cr_max_=180;
    int w_cb_min_=85,  w_cb_max_=135;
    int w_visualize_ = 1;

    // Yellow Mask: HLS + HSV
    int y_roi_bottom_ = 10, y_roi_top_ = 40;
    int y_hls_h_min_=0,   y_hls_h_max_=180;
    int y_hls_l_min_=200, y_hls_l_max_=255;
    int y_hls_s_min_=0,   y_hls_s_max_=80;
    int y_hsv_h_min_=0,   y_hsv_h_max_=180;
    int y_hsv_s_min_=0,   y_hsv_s_max_=60;
    int y_hsv_v_min_=200, y_hsv_v_max_=255;
    int y_visualize_ = 1;

    // Yellow Image (Post)
    int yi_open_kernel_=3, yi_close_kernel_=5;
    int yi_open_iters_=1, yi_close_iters_=1;
    int yi_sobel_ksize_=3;
    int yi_sobel_scale_slider_=10; // 1.0
    int yi_sobel_delta_slider_=0;  // 0.0
    int yi_sobel_thresh_=30;

    // ==== 콜백 ====
    void image_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
            std::lock_guard<std::mutex> lk(mtx_);
            bgr.copyTo(last_bgr_);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "cv_bridge error: %s", e.what());
        }
    }

    void on_timer() {
        // 키 처리
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) { // ESC or q
            rclcpp::shutdown();
            return;
        } else if (key == 's') {
            if (save_config()) {
                RCLCPP_INFO(this->get_logger(), "Config saved to %s", config_path_.c_str());
            }
        } else if (key == 'r') {
            if (reload_config()) {
                RCLCPP_INFO(this->get_logger(), "Config reloaded from %s", config_path_.c_str());
                // 로드된 값 → 슬라이더 변수 반영 → 트랙바 위치 갱신
                load_slider_values_from_cfg();
                set_trackbars_from_sliders();
            }
        }

        // 최신 프레임
        cv::Mat bgr;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (last_bgr_.empty()) return;
            bgr = last_bgr_.clone();
        }

        // 슬라이더 → cfg
        sync_cfg_from_sliders();

        // ---- 마스킹 파이프라인 ----
        cv::Mat wmask, wvis;
        lane::preprocess::WMaskProcessor(bgr, cfg_.white_mask, wmask, cfg_.white_mask.visualize ? &wvis : nullptr);

        cv::Mat ymask, yvis;
        lane::preprocess::YMaskProcessor(bgr, cfg_.yellow_mask, ymask, cfg_.yellow_mask.visualize ? &yvis : nullptr);

        cv::Mat ypost;
        lane::preprocess::YImageProcessor(ymask, cfg_.yellow_image, ypost);

        // ---- 직선/오프셋 계산 & 시각화 ----
        cv::Mat viz = bgr.clone();  // 원본 위에 그리기
        update_detect_params_from_node(); // 파라미터 최신화(옵션)

        // 화이트 라인 (wmask)
        lane::offset::LineFit wline = lane::offset::WLineCalculate(wmask, viz, dp_);
        // 옐로 라인 (후처리 결과가 있으면 그걸 사용, 아니면 ymask)
        lane::offset::OffsetViz ov;
        const cv::Mat& y_input = !ypost.empty() ? ypost : ymask;
        lane::offset::LineFit yline = lane::offset::YLineCalculate(y_input, viz, dp_, &wline, &ov);

        // ---- 표시 ----
        if (!wvis.empty()) cv::imshow(WIN_WHITE, wvis); else cv::imshow(WIN_WHITE, bgr);
        if (!yvis.empty()) cv::imshow(WIN_YELLOW, yvis); else cv::imshow(WIN_YELLOW, bgr);
        if (!ypost.empty()) cv::imshow(WIN_POST, ypost);
        cv::imshow(WIN_VIZ, viz);
    }

    // ==== cfg → 슬라이더 값 ====
    void load_slider_values_from_cfg() {
        // White (Lab + YCrCb)
        w_roi_bottom_ = cfg_.white_mask.roi_bottom_pct;
        w_roi_top_    = cfg_.white_mask.roi_top_pct;

        w_lab_l_min_ = cfg_.white_mask.lab_l_min; w_lab_l_max_ = cfg_.white_mask.lab_l_max;
        w_lab_a_min_ = cfg_.white_mask.lab_a_min; w_lab_a_max_ = cfg_.white_mask.lab_a_max;
        w_lab_b_min_ = cfg_.white_mask.lab_b_min; w_lab_b_max_ = cfg_.white_mask.lab_b_max;

        w_y_min_  = cfg_.white_mask.y_min;  w_y_max_  = cfg_.white_mask.y_max;
        w_cr_min_ = cfg_.white_mask.cr_min; w_cr_max_ = cfg_.white_mask.cr_max;
        w_cb_min_ = cfg_.white_mask.cb_min; w_cb_max_ = cfg_.white_mask.cb_max;

        w_visualize_ = cfg_.white_mask.visualize ? 1 : 0;

        // Yellow (HLS + HSV)
        y_roi_bottom_ = cfg_.yellow_mask.roi_bottom_pct;
        y_roi_top_    = cfg_.yellow_mask.roi_top_pct;

        y_hls_h_min_ = cfg_.yellow_mask.hls_h_min; y_hls_h_max_ = cfg_.yellow_mask.hls_h_max;
        y_hls_l_min_ = cfg_.yellow_mask.hls_l_min; y_hls_l_max_ = cfg_.yellow_mask.hls_l_max;
        y_hls_s_min_ = cfg_.yellow_mask.hls_s_min; y_hls_s_max_ = cfg_.yellow_mask.hls_s_max;

        y_hsv_h_min_ = cfg_.yellow_mask.hsv_h_min; y_hsv_h_max_ = cfg_.yellow_mask.hsv_h_max;
        y_hsv_s_min_ = cfg_.yellow_mask.hsv_s_min; y_hsv_s_max_ = cfg_.yellow_mask.hsv_s_max;
        y_hsv_v_min_ = cfg_.yellow_mask.hsv_v_min; y_hsv_v_max_ = cfg_.yellow_mask.hsv_v_max;

        y_visualize_ = cfg_.yellow_mask.visualize ? 1 : 0;

        // Yellow Post
        yi_open_kernel_  = cfg_.yellow_image.open_kernel;
        yi_close_kernel_ = cfg_.yellow_image.close_kernel;
        yi_open_iters_   = cfg_.yellow_image.open_iters;
        yi_close_iters_  = cfg_.yellow_image.close_iters;

        yi_sobel_ksize_        = cfg_.yellow_image.sobel_ksize;
        yi_sobel_scale_slider_ = to_scale_slider(cfg_.yellow_image.sobel_scale);
        yi_sobel_delta_slider_ = to_delta_slider(cfg_.yellow_image.sobel_delta);
        yi_sobel_thresh_       = cfg_.yellow_image.sobel_thresh;
    }

    // ==== 트랙바/윈도우 생성 ====
    void create_windows_and_trackbars() {
        // cfg → 슬라이더
        load_slider_values_from_cfg();

        // 창
        cv::namedWindow(WIN_WHITE,  cv::WINDOW_NORMAL);
        cv::namedWindow(WIN_YELLOW, cv::WINDOW_NORMAL);
        cv::namedWindow(WIN_POST,   cv::WINDOW_NORMAL);
        cv::namedWindow(WIN_VIZ,    cv::WINDOW_NORMAL);  // ✅ 추가
        cv::resizeWindow(WIN_WHITE,  640, 360);
        cv::resizeWindow(WIN_YELLOW, 640, 360);
        cv::resizeWindow(WIN_POST,   640, 360);
        cv::resizeWindow(WIN_VIZ,    800, 450);

        // White Trackbars (Lab + YCrCb)
        cv::createTrackbar("roi_bottom(%)", WIN_WHITE, &w_roi_bottom_, 100);
        cv::createTrackbar("roi_top(%)",    WIN_WHITE, &w_roi_top_,    100);

        cv::createTrackbar("Lab L min", WIN_WHITE, &w_lab_l_min_, 255);
        cv::createTrackbar("Lab L max", WIN_WHITE, &w_lab_l_max_, 255);
        cv::createTrackbar("Lab a min", WIN_WHITE, &w_lab_a_min_, 255);
        cv::createTrackbar("Lab a max", WIN_WHITE, &w_lab_a_max_, 255);
        cv::createTrackbar("Lab b min", WIN_WHITE, &w_lab_b_min_, 255);
        cv::createTrackbar("Lab b max", WIN_WHITE, &w_lab_b_max_, 255);

        cv::createTrackbar("Y min",  WIN_WHITE, &w_y_min_,  255);
        cv::createTrackbar("Y max",  WIN_WHITE, &w_y_max_,  255);
        cv::createTrackbar("Cr min", WIN_WHITE, &w_cr_min_, 255);
        cv::createTrackbar("Cr max", WIN_WHITE, &w_cr_max_, 255);
        cv::createTrackbar("Cb min", WIN_WHITE, &w_cb_min_, 255);
        cv::createTrackbar("Cb max", WIN_WHITE, &w_cb_max_, 255);

        cv::createTrackbar("visualize(0/1)", WIN_WHITE, &w_visualize_, 1);

        // Yellow Trackbars (HLS + HSV)
        cv::createTrackbar("roi_bottom(%)", WIN_YELLOW, &y_roi_bottom_, 100);
        cv::createTrackbar("roi_top(%)",    WIN_YELLOW, &y_roi_top_,    100);

        cv::createTrackbar("HLS H min", WIN_YELLOW, &y_hls_h_min_, 180);
        cv::createTrackbar("HLS H max", WIN_YELLOW, &y_hls_h_max_, 180);
        cv::createTrackbar("HLS L min", WIN_YELLOW, &y_hls_l_min_, 255);
        cv::createTrackbar("HLS L max", WIN_YELLOW, &y_hls_l_max_, 255);
        cv::createTrackbar("HLS S min", WIN_YELLOW, &y_hls_s_min_, 255);
        cv::createTrackbar("HLS S max", WIN_YELLOW, &y_hls_s_max_, 255);

        cv::createTrackbar("HSV H min", WIN_YELLOW, &y_hsv_h_min_, 180);
        cv::createTrackbar("HSV H max", WIN_YELLOW, &y_hsv_h_max_, 180);
        cv::createTrackbar("HSV S min", WIN_YELLOW, &y_hsv_s_min_, 255);
        cv::createTrackbar("HSV S max", WIN_YELLOW, &y_hsv_s_max_, 255);
        cv::createTrackbar("HSV V min", WIN_YELLOW, &y_hsv_v_min_, 255);
        cv::createTrackbar("HSV V max", WIN_YELLOW, &y_hsv_v_max_, 255);

        cv::createTrackbar("visualize(0/1)", WIN_YELLOW, &y_visualize_, 1);

        // Yellow Post
        cv::createTrackbar("Open kernel",  WIN_POST, &yi_open_kernel_,  31);
        cv::createTrackbar("Open iters",   WIN_POST, &yi_open_iters_,   10);
        cv::createTrackbar("Close kernel", WIN_POST, &yi_close_kernel_, 31);
        cv::createTrackbar("Close iters",  WIN_POST, &yi_close_iters_,  10);

        cv::createTrackbar("Sobel ksize(1-7)",  WIN_POST, &yi_sobel_ksize_, 7);
        cv::createTrackbar("Sobel scale(x0.1)", WIN_POST, &yi_sobel_scale_slider_, SCALE_SLIDER_MAX);
        cv::createTrackbar("Sobel delta",       WIN_POST, &yi_sobel_delta_slider_, DELTA_SLIDER_MAX);
        cv::createTrackbar("Sobel thresh",      WIN_POST, &yi_sobel_thresh_, 255);

        // 트랙바 위치 세팅
        set_trackbars_from_sliders();
    }

    // ==== 트랙바 위치 세팅 ====
    void set_trackbars_from_sliders() {
      // WHITE (Lab + YCrCb)
      cv::setTrackbarPos("roi_bottom(%)", WIN_WHITE, w_roi_bottom_);
      cv::setTrackbarPos("roi_top(%)",    WIN_WHITE, w_roi_top_);
      cv::setTrackbarPos("Lab L min", WIN_WHITE, w_lab_l_min_);
      cv::setTrackbarPos("Lab L max", WIN_WHITE, w_lab_l_max_);
      cv::setTrackbarPos("Lab a min", WIN_WHITE, w_lab_a_min_);
      cv::setTrackbarPos("Lab a max", WIN_WHITE, w_lab_a_max_);
      cv::setTrackbarPos("Lab b min", WIN_WHITE, w_lab_b_min_);
      cv::setTrackbarPos("Lab b max", WIN_WHITE, w_lab_b_max_);
      cv::setTrackbarPos("Y min",  WIN_WHITE, w_y_min_);
      cv::setTrackbarPos("Y max",  WIN_WHITE, w_y_max_);
      cv::setTrackbarPos("Cr min", WIN_WHITE, w_cr_min_);
      cv::setTrackbarPos("Cr max", WIN_WHITE, w_cr_max_);
      cv::setTrackbarPos("Cb min", WIN_WHITE, w_cb_min_);
      cv::setTrackbarPos("Cb max", WIN_WHITE, w_cb_max_);
      cv::setTrackbarPos("visualize(0/1)", WIN_WHITE, w_visualize_);

      // YELLOW (HLS + HSV)
      cv::setTrackbarPos("roi_bottom(%)", WIN_YELLOW, y_roi_bottom_);
      cv::setTrackbarPos("roi_top(%)",    WIN_YELLOW, y_roi_top_);
      cv::setTrackbarPos("HLS H min", WIN_YELLOW, y_hls_h_min_);
      cv::setTrackbarPos("HLS H max", WIN_YELLOW, y_hls_h_max_);
      cv::setTrackbarPos("HLS L min", WIN_YELLOW, y_hls_l_min_);
      cv::setTrackbarPos("HLS L max", WIN_YELLOW, y_hls_l_max_);
      cv::setTrackbarPos("HLS S min", WIN_YELLOW, y_hls_s_min_);
      cv::setTrackbarPos("HLS S max", WIN_YELLOW, y_hls_s_max_);
      cv::setTrackbarPos("HSV H min", WIN_YELLOW, y_hsv_h_min_);
      cv::setTrackbarPos("HSV H max", WIN_YELLOW, y_hsv_h_max_);
      cv::setTrackbarPos("HSV S min", WIN_YELLOW, y_hsv_s_min_);
      cv::setTrackbarPos("HSV S max", WIN_YELLOW, y_hsv_s_max_);
      cv::setTrackbarPos("HSV V min", WIN_YELLOW, y_hsv_v_min_);
      cv::setTrackbarPos("HSV V max", WIN_YELLOW, y_hsv_v_max_);
      cv::setTrackbarPos("visualize(0/1)", WIN_YELLOW, y_visualize_);

      // Yellow Post
      cv::setTrackbarPos("Open kernel",       WIN_POST, yi_open_kernel_);
      cv::setTrackbarPos("Open iters",        WIN_POST, yi_open_iters_);
      cv::setTrackbarPos("Close kernel",      WIN_POST, yi_close_kernel_);
      cv::setTrackbarPos("Close iters",       WIN_POST, yi_close_iters_);
      cv::setTrackbarPos("Sobel ksize(1-7)",  WIN_POST, yi_sobel_ksize_);
      cv::setTrackbarPos("Sobel scale(x0.1)", WIN_POST, yi_sobel_scale_slider_);
      cv::setTrackbarPos("Sobel delta",       WIN_POST, yi_sobel_delta_slider_);
      cv::setTrackbarPos("Sobel thresh",      WIN_POST, yi_sobel_thresh_);
    }

    // ==== 슬라이더 → cfg 반영 ====
    void sync_cfg_from_sliders() {
        auto fix_pair = [](int& mn, int& mx) { if (mn > mx) std::swap(mn, mx); };

        // White
        fix_pair(w_roi_bottom_, w_roi_top_);
        cfg_.white_mask.roi_bottom_pct = clampi(w_roi_bottom_, 0, 100);
        cfg_.white_mask.roi_top_pct    = clampi(w_roi_top_,    0, 100);

        fix_pair(w_lab_l_min_, w_lab_l_max_);
        fix_pair(w_lab_a_min_, w_lab_a_max_);
        fix_pair(w_lab_b_min_, w_lab_b_max_);
        fix_pair(w_y_min_,     w_y_max_);
        fix_pair(w_cr_min_,    w_cr_max_);
        fix_pair(w_cb_min_,    w_cb_max_);

        cfg_.white_mask.lab_l_min = clampi(w_lab_l_min_, 0, 255);
        cfg_.white_mask.lab_l_max = clampi(w_lab_l_max_, 0, 255);
        cfg_.white_mask.lab_a_min = clampi(w_lab_a_min_, 0, 255);
        cfg_.white_mask.lab_a_max = clampi(w_lab_a_max_, 0, 255);
        cfg_.white_mask.lab_b_min = clampi(w_lab_b_min_, 0, 255);
        cfg_.white_mask.lab_b_max = clampi(w_lab_b_max_, 0, 255);

        cfg_.white_mask.y_min  = clampi(w_y_min_,  0, 255);
        cfg_.white_mask.y_max  = clampi(w_y_max_,  0, 255);
        cfg_.white_mask.cr_min = clampi(w_cr_min_, 0, 255);
        cfg_.white_mask.cr_max = clampi(w_cr_max_, 0, 255);
        cfg_.white_mask.cb_min = clampi(w_cb_min_, 0, 255);
        cfg_.white_mask.cb_max = clampi(w_cb_max_, 0, 255);

        cfg_.white_mask.visualize = (w_visualize_ != 0);

        // Yellow
        fix_pair(y_roi_bottom_, y_roi_top_);
        cfg_.yellow_mask.roi_bottom_pct = clampi(y_roi_bottom_, 0, 100);
        cfg_.yellow_mask.roi_top_pct    = clampi(y_roi_top_,    0, 100);

        fix_pair(y_hls_h_min_, y_hls_h_max_);
        fix_pair(y_hls_l_min_, y_hls_l_max_);
        fix_pair(y_hls_s_min_, y_hls_s_max_);
        fix_pair(y_hsv_h_min_, y_hsv_h_max_);
        fix_pair(y_hsv_s_min_, y_hsv_s_max_);
        fix_pair(y_hsv_v_min_, y_hsv_v_max_);

        cfg_.yellow_mask.hls_h_min = clampi(y_hls_h_min_, 0, 180);
        cfg_.yellow_mask.hls_h_max = clampi(y_hls_h_max_, 0, 180);
        cfg_.yellow_mask.hls_l_min = clampi(y_hls_l_min_, 0, 255);
        cfg_.yellow_mask.hls_l_max = clampi(y_hls_l_max_, 0, 255);
        cfg_.yellow_mask.hls_s_min = clampi(y_hls_s_min_, 0, 255);
        cfg_.yellow_mask.hls_s_max = clampi(y_hls_s_max_, 0, 255);

        cfg_.yellow_mask.hsv_h_min = clampi(y_hsv_h_min_, 0, 180);
        cfg_.yellow_mask.hsv_h_max = clampi(y_hsv_h_max_, 0, 180);
        cfg_.yellow_mask.hsv_s_min = clampi(y_hsv_s_min_, 0, 255);
        cfg_.yellow_mask.hsv_s_max = clampi(y_hsv_s_max_, 0, 255);
        cfg_.yellow_mask.hsv_v_min = clampi(y_hsv_v_min_, 0, 255);
        cfg_.yellow_mask.hsv_v_max = clampi(y_hsv_v_max_, 0, 255);

        cfg_.yellow_mask.visualize = (y_visualize_ != 0);

        // Yellow Post
        cfg_.yellow_image.open_kernel  = std::max(1, yi_open_kernel_);
        cfg_.yellow_image.close_kernel = std::max(1, yi_close_kernel_);
        cfg_.yellow_image.open_iters   = std::max(0, yi_open_iters_);
        cfg_.yellow_image.close_iters  = std::max(0, yi_close_iters_);
        cfg_.yellow_image.open_kernel  = odd_or_next(cfg_.yellow_image.open_kernel);
        cfg_.yellow_image.close_kernel = odd_or_next(cfg_.yellow_image.close_kernel);

        // Sobel
        yi_sobel_ksize_ = clampi(yi_sobel_ksize_, 1, 7);
        cfg_.yellow_image.sobel_ksize  = odd_or_next(yi_sobel_ksize_);
        cfg_.yellow_image.sobel_scale  = from_scale_slider(yi_sobel_scale_slider_);
        cfg_.yellow_image.sobel_delta  = from_delta_slider(yi_sobel_delta_slider_);
        cfg_.yellow_image.sobel_thresh = clampi(yi_sobel_thresh_, 0, 255);
    }

    // ==== DetectParams 파라미터 ====
    void declare_detect_params() {
        this->declare_parameter<int>("det.canny_low", 0);
        this->declare_parameter<int>("det.canny_high", 0);
        this->declare_parameter<int>("det.canny_aperture", 3);

        this->declare_parameter<double>("det.rho", 1.0);
        this->declare_parameter<double>("det.theta_deg", 1.0); // degrees
        this->declare_parameter<int>("det.hough_thresh", 30);
        this->declare_parameter<double>("det.min_line_length", 60.0);
        this->declare_parameter<double>("det.max_line_gap", 10.0);

        this->declare_parameter<double>("det.start_x_min_frac", 0.55);
        this->declare_parameter<double>("det.start_x_max_frac", 0.98);
        this->declare_parameter<double>("det.start_y_min_frac", 0.60);
        this->declare_parameter<double>("det.start_y_max_frac", 0.98);

        update_detect_params_from_node();
    }

    void update_detect_params_from_node() {
        dp_.canny_low  = this->get_parameter("det.canny_low").as_int();
        dp_.canny_high = this->get_parameter("det.canny_high").as_int();
        dp_.canny_aperture = this->get_parameter("det.canny_aperture").as_int();

        dp_.rho   = this->get_parameter("det.rho").as_double();
        double theta_deg = this->get_parameter("det.theta_deg").as_double();
        dp_.theta = theta_deg * CV_PI / 180.0;
        dp_.hough_thresh = this->get_parameter("det.hough_thresh").as_int();
        dp_.min_line_length = this->get_parameter("det.min_line_length").as_double();
        dp_.max_line_gap    = this->get_parameter("det.max_line_gap").as_double();

        dp_.start_x_min_frac = this->get_parameter("det.start_x_min_frac").as_double();
        dp_.start_x_max_frac = this->get_parameter("det.start_x_max_frac").as_double();
        dp_.start_y_min_frac = this->get_parameter("det.start_y_min_frac").as_double();
        dp_.start_y_max_frac = this->get_parameter("det.start_y_max_frac").as_double();
    }

    // 저장/리로드
    bool save_config() {
        std::string err;
        if (!lane::config::io::SaveToFile(config_path_, cfg_, &err)) {
            RCLCPP_ERROR(this->get_logger(), "Save failed: %s", err.c_str());
            return false;
        }
        return true;
    }

    bool reload_config() {
        lane::config::LaneConfig loaded;
        std::string err;
        if (!lane::config::io::LoadFromFile(config_path_, loaded, &err)) {
            RCLCPP_ERROR(this->get_logger(), "Reload failed: %s", err.c_str());
            return false;
        }
        cfg_ = std::move(loaded);
        return true;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MaskTunerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
