/**
 * @file object_detection.cpp
 * @brief 줌(확대) 없이 원본 비율 유지. 입력(640x360 등) 전체를 그대로 BEV로 사용(동일 크기, 항등 투영).
 *        lane_detection이 퍼블리시한 트랙 중앙선(x = m*y + b)을 따라 ‘중앙 밴드(corridor)’를 y별로 생성하고,
 *        밴드 내 엣지 점유율/최대 컨투어 면적으로 장애물 판정.
 *
 * 입력
 *  - /resized_image (sensor_msgs/msg/Image, BGR8, 보통 640x360)
 *  - /lane_fit      (std_msgs/msg/Float32MultiArray, data=[m, b], BEV 좌표계에서 x = m*y + b)
 *
 * 출력
 *  - /object_info (std_msgs/msg/Int16): -2(회피 필요), -1(계속 주행)
 *
 * 파라미터(ROS2)
 *  - camera_topic(string)           : 입력 이미지 토픽명 (기본 "/resized_image")
 *  - lane_fit_topic(string)         : 중앙선 파라미터 토픽명 (기본 "/lane_fit")
 *  - roi_top_ratio(double)          : 판정용 유효 y-구간 상단 비율(0~1, 기본 0.55)
 *  - roi_bottom_ratio(double)       : 판정용 유효 y-구간 하단 비율(0~1, 기본 0.95)
 *  - corridor_half_px(int)          : 중앙선 좌우 밴드 반폭(px)
 *  - canny_low/high(int)            : Canny 임계
 *  - morph_size(int)                : 모폴로지 커널 크기(홀수 권장)
 *  - block_ratio_th(double)         : 밴드 내 엣지 점유율 임계(0~1)
 *  - min_block_area_px(int)         : 밴드 내 최대 컨투어 면적 임계(px)
 *  - stable_frames_on/off(int)      : 히스테리시스 프레임 수
 *  - debug_view(bool)               : 디버그 오버레이/창 출력
 *
 * 설계 핵심
 *  - BEV 크기를 ‘입력 프레임 크기’로 동적으로 설정하고, Homography는 항등(전체 화면→전체 화면)으로 구성.
 *  - 따라서 확대(zoom) 없음. ROI 사다리꼴을 BEV 전체로 ‘늘리지’ 않음.
 *  - 판정은 ROI 비율(상단/하단)로 y-구간을 제한하고, 그 구간에서만 중앙선 밴드 마스크를 생성.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <chrono>

using std::placeholders::_1;
using namespace cv;
using namespace std::chrono_literals;

class ObjectDetectionNode : public rclcpp::Node {
public:
  ObjectDetectionNode() : Node("object_detector_node") {
    // -------------------- 파라미터 로딩 --------------------
    camera_topic_      = declare_parameter<std::string>("camera_topic", "/resized_image");
    lane_fit_topic_    = declare_parameter<std::string>("lane_fit_topic", "/lane_fit");

    roi_top_ratio_     = declare_parameter<double>("roi_top_ratio",    0.55);
    roi_bottom_ratio_  = declare_parameter<double>("roi_bottom_ratio", 0.95);
    corridor_half_px_  = declare_parameter<int>("corridor_half_px",    90);

    canny_low_         = declare_parameter<int>("canny_low",  60);
    canny_high_        = declare_parameter<int>("canny_high", 140);
    morph_size_        = declare_parameter<int>("morph_size", 5);

    block_ratio_th_    = declare_parameter<double>("block_ratio_th",   0.12);
    min_block_area_px_ = declare_parameter<int>("min_block_area_px",   1800);
    stable_on_         = declare_parameter<int>("stable_frames_on",    3);
    stable_off_        = declare_parameter<int>("stable_frames_off",   3);

    debug_view_        = declare_parameter<bool>("debug_view", false);

    // -------------------- ROS I/F --------------------
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObjectDetectionNode::onImage, this, _1));

    lane_fit_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      lane_fit_topic_, 10, std::bind(&ObjectDetectionNode::onLaneFit, this, _1));

    info_pub_ = create_publisher<std_msgs::msg::Int16>("/object_info", 10);

    RCLCPP_INFO(get_logger(),
      "[object_detection] ready. camera=%s lane_fit=%s | /object_info (-1 clear / -2 blocked)",
      camera_topic_.c_str(), lane_fit_topic_.c_str());
  }

  ~ObjectDetectionNode() {
    if (window_created_) {
      try { cv::destroyWindow("object_detection/BEV"); } catch (...) {}
    }
  }

private:
  // ==================== 구성 파라미터/상태 ====================
  std::string camera_topic_, lane_fit_topic_;
  double roi_top_ratio_{0.55}, roi_bottom_ratio_{0.95};
  int corridor_half_px_{90};

  int canny_low_{60}, canny_high_{140}, morph_size_{5};

  double block_ratio_th_{0.12};
  int min_block_area_px_{1800};
  int stable_on_{3}, stable_off_{3};
  bool debug_view_{false};

  // 투영행렬(Image→BEV) 및 BEV 크기(입력 크기와 동일, 런타임 설정)
  Mat H_;                 // 항등 투영행렬
  Size bev_size_{0, 0};   // 첫 프레임에서 설정, 이후 입력 크기 변경 시 갱신

  // lane_fit (트랙 중앙선 직선 파라미터 x = m*y + b)
  bool  has_fit_{false};
  float m_{0.f}, b_{0.f};                 // b 초기값은 첫 프레임 수신 후 중앙으로 설정
  bool  b_initialized_{false};
  rclcpp::Time last_fit_time_;
  rclcpp::Duration fit_timeout_{500ms};

  // 안정화
  int blocked_streak_{0}, clear_streak_{0};
  bool blocked_state_{false};

  // 디버그 창
  bool window_created_{false};

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lane_fit_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr info_pub_;

  // ==================== 유틸 ====================
  inline int clamp_int(int v, int lo, int hi) const { return v < lo ? lo : (v > hi ? hi : v); }

  // ==================== 함수: lane_fit 수신 콜백 ====================
  /**
   * @brief lane_detection이 퍼블리시한 중앙선 파라미터(m, b) 수신.
   *        최근 수신 시간을 기록하여 타임아웃 시 경고만 출력하고 마지막 값 유지.
   */
  void onLaneFit(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
      m_ = msg->data[0];
      b_ = msg->data[1];
      has_fit_ = true;
      last_fit_time_ = now();
    }
  }

  // ==================== 함수: 항등 투영행렬 구성 ====================
  /**
   * @brief 입력 전체(0,0)-(W-1,H-1)을 BEV 전체(동일 크기)로 매핑하는 항등 Homography 계산.
   *        확대(zoom)가 발생하지 않도록 함.
   */
  void buildIdentityHomography(int W, int H) {
    bev_size_ = Size(W, H);

    Point2f src[4] = {
      Point2f(0.f,   0.f),
      Point2f(W-1.f, 0.f),
      Point2f(W-1.f, H-1.f),
      Point2f(0.f,   H-1.f)
    };
    Point2f dst[4] = {
      Point2f(0.f,   0.f),
      Point2f(W-1.f, 0.f),
      Point2f(W-1.f, H-1.f),
      Point2f(0.f,   H-1.f)
    };

    H_ = getPerspectiveTransform(src, dst);
    RCLCPP_INFO(get_logger(), "Homography built: identity, BEV size = %dx%d", bev_size_.width, bev_size_.height);
  }

  // ==================== 함수: 중앙선 기반 밴드 마스크 생성(ROI y-구간 제한) ====================
  /**
   * @brief lane_fit(x = m*y + b)을 사용해 y=y_top..y_bottom 각 줄에서 중심 x를 계산.
   *        그 중심을 기준으로 반폭 corridor_half_px_ 구간을 255로 채워 밴드 마스크를 생성.
   *        lane_fit 타임아웃 시 마지막 값 사용, 수신 전에는 화면 중앙선으로 폴백.
   *        y-구간은 roi_top_ratio/roi_bottom_ratio로 제한.
   */
  void makeCorridorMaskFromFit(Mat& mask) {
    mask = Mat::zeros(bev_size_, CV_8UC1);
    const int Wb = bev_size_.width, Hb = bev_size_.height;

    // 폴백 b 초기화(첫 프레임 시 중앙선)
    if (!b_initialized_ && Wb > 0) {
      b_ = static_cast<float>(Wb / 2);
      b_initialized_ = true;
    }

    const int y_top    = clamp_int(static_cast<int>(Hb * roi_top_ratio_),    0, Hb-1);
    const int y_bottom = clamp_int(static_cast<int>(Hb * roi_bottom_ratio_), 0, Hb-1);

    const bool fit_ok = has_fit_;
    if (has_fit_ && (now() - last_fit_time_) > fit_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "lane_fit timeout. using last (m,b)=%.5f,%.1f", m_, b_);
    }

    for (int y = y_top; y <= y_bottom; ++y) {
      int x_center = fit_ok ? (int)std::lround(m_ * y + b_) : (Wb / 2);
      x_center = clamp_int(x_center, 0, Wb - 1);

      int x0 = clamp_int(x_center - corridor_half_px_, 0, Wb - 1);
      int x1 = clamp_int(x_center + corridor_half_px_, 0, Wb - 1);
      if (x1 >= x0) {
        mask.row(y).colRange(x0, x1).setTo(255);
      }
    }
  }

  // ==================== 함수: 전처리(엣지+모폴로지) ====================
  /**
   * @brief BEV BGR → GRAY → Blur → Canny → Morphology(OPEN/CLOSE)로 엣지를 정리.
   */
  void preprocessEdges(const Mat& bev_bgr, Mat& edges) const {
    Mat gray; cvtColor(bev_bgr, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(5,5), 0);
    Canny(gray, edges, canny_low_, canny_high_);
    if (morph_size_ > 1) {
      Mat k = getStructuringElement(MORPH_RECT, Size(morph_size_, morph_size_));
      morphologyEx(edges, edges, MORPH_OPEN,  k);
      morphologyEx(edges, edges, MORPH_CLOSE, k);
    }
  }

  // ==================== 함수: 막힘 판정(단일 프레임) ====================
  /**
   * @brief 밴드 내 엣지 점유율과 최대 컨투어 면적으로 프레임 단위 ‘막힘’ 여부를 판단.
   */
  bool isBlockedFrame(const Mat& edges, const Mat& corridor_mask,
                      double& occ_ratio_out, double& max_contour_area_out) const
  {
    Mat occ; bitwise_and(edges, corridor_mask, occ);

    const double band_area  = static_cast<double>(countNonZero(corridor_mask));
    const double occ_pixels = static_cast<double>(countNonZero(occ));
    occ_ratio_out = (band_area > 1.0) ? (occ_pixels / band_area) : 0.0;

    std::vector<std::vector<Point>> contours;
    findContours(occ, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    double max_area = 0.0;
    for (const auto& c : contours) {
      double a = contourArea(c);
      if (a > max_area) max_area = a;
    }
    max_contour_area_out = max_area;

    const bool by_ratio = (occ_ratio_out >= block_ratio_th_);
    const bool by_area  = (max_contour_area_out >= static_cast<double>(min_block_area_px_));
    return by_ratio || by_area;
  }

  // ==================== 함수: 상태 안정화(Hysteresis) ====================
  /**
   * @brief 프레임 단위 결과를 연속 프레임 카운터로 안정화하여 최종 상태 갱신.
   */
  void updateStableState(bool frameBlocked) {
    if (frameBlocked) { ++blocked_streak_; clear_streak_ = 0; }
    else              { ++clear_streak_;  blocked_streak_ = 0; }

    const bool turn_on  = (!blocked_state_ && blocked_streak_ >= stable_on_);
    const bool turn_off = ( blocked_state_ && clear_streak_   >= stable_off_);

    if (turn_on)  blocked_state_ = true;
    if (turn_off) blocked_state_ = false;
  }

  // ==================== 콜백: 이미지 처리 메인 파이프라인 ====================
  /**
   * @brief 입력 → BEV(항등) → 엣지 전처리 → 중앙선 기반 밴드 마스크(ROI y-구간 제한) → 막힘 판단 → 퍼블리시 → 디버그
   */
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    // --- ROS 이미지 → CV BGR ---
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
      return;
    }
    Mat frame = cv_ptr->image;
    if (frame.empty()) return;

    // --- 0) 항등 Homography 초기화/갱신(입력 크기 변화 대응) ---
    if (bev_size_.width != frame.cols || bev_size_.height != frame.rows || H_.empty()) {
      buildIdentityHomography(frame.cols, frame.rows);
      // b 초기 중앙값도 프레임 폭에 맞춰 갱신
      b_initialized_ = false;
    }

    // --- 1) BEV 투영(항등, 확대 없음) ---
    Mat bev;
    warpPerspective(frame, bev, H_, bev_size_, INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0));

    // --- 2) 엣지 전처리 ---
    Mat edges;
    preprocessEdges(bev, edges);

    // --- 3) 중앙선 기반 밴드 마스크(ROI y-구간 제한) ---
    Mat band_mask;
    makeCorridorMaskFromFit(band_mask);

    // --- 4) 막힘 판정(프레임 단위) ---
    double occ_ratio = 0.0, max_area = 0.0;
    const bool blocked_now = isBlockedFrame(edges, band_mask, occ_ratio, max_area);

    // --- 5) 상태 안정화 ---
    updateStableState(blocked_now);

    // --- 6) 퍼블리시 ---
    std_msgs::msg::Int16 out; out.data = blocked_state_ ? -2 : -1;
    info_pub_->publish(out);

    // --- 7) 디버그 시각화 ---
    if (debug_view_) {
      if (!window_created_) {
        // 이미지 크기=창 크기. 확대한 느낌 방지.
        namedWindow("object_detection/BEV", WINDOW_AUTOSIZE);
        window_created_ = true;
      }

      // 중앙선 폴리라인(트랙 중앙)
      Mat dbg = bev.clone();
      const int Hb = bev.rows, Wb = bev.cols;
      const bool fit_ok = has_fit_;
      if (has_fit_ && (now() - last_fit_time_) > fit_timeout_) {
        // 이미 경고 출력
      }
      std::vector<Point> centerline; centerline.reserve(Hb);
      for (int y = 0; y < Hb; ++y) {
        int x = fit_ok ? (int)std::lround(m_ * y + b_) : (Wb / 2);
        x = clamp_int(x, 0, Wb - 1);
        centerline.emplace_back(x, y);
      }
      for (size_t i = 1; i < centerline.size(); ++i) {
        line(dbg, centerline[i-1], centerline[i], Scalar(0,255,255), 2); // 노랑
      }

      // 밴드 오버레이 + 엣지
      Mat overlay = Mat::zeros(dbg.size(), CV_8UC3);
      overlay.setTo(Scalar(255, 0, 0), band_mask); // 파랑
      Mat edges_bgr; cvtColor(edges, edges_bgr, COLOR_GRAY2BGR);

      Mat blended;
      addWeighted(dbg, 1.0, overlay, 0.35, 0.0, blended);
      addWeighted(blended, 1.0, edges_bgr, 0.35, 0.0, blended);

      // 정보 표시
      putText(blended,
              format("occ=%.3f area=%.0f thr=%.2f/%d state=%s",
                     occ_ratio, max_area, block_ratio_th_, min_block_area_px_,
                     blocked_state_ ? "BLOCKED(-2)" : "CLEAR(-1)"),
              Point(10, 24), FONT_HERSHEY_SIMPLEX, 0.6,
              blocked_state_ ? Scalar(0,0,255) : Scalar(0,200,0), 2);

      imshow("object_detection/BEV", blended);
      waitKey(1);
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
