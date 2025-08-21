// src/object_detection.cpp
// 입력
//  - /scan : sensor_msgs/msg/LaserScan
//  - /resized_image : sensor_msgs/msg/Image (BGR8)
// 출력
//  - /object_info : std_msgs/msg/Float32MultiArray, [exists, min_dist, angle, span, cluster_size]
// 시각화
//  - "OBJECT DEBUG" : exists / distance / cluster_size
//  - "CAMERA VIEW" : 입력 영상 그대로 출력

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>          // <-- 추가
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <limits>
#include <cmath>
#include <vector>
#include <iomanip>
#include <sstream>

using std::placeholders::_1;

class ObjectDetectionNode : public rclcpp::Node {
public:
  ObjectDetectionNode() : Node("object_node") {
    // 파라미터
    front_fov_deg_       = this->declare_parameter<double>("front_fov_deg", 7.0);
    range_min_m_         = this->declare_parameter<double>("range_min_m",   0.05);
    range_max_m_         = this->declare_parameter<double>("range_max_m",   2.0);
    cluster_epsilon_m_   = this->declare_parameter<double>("cluster_epsilon_m", 0.20);
    min_cluster_points_  = this->declare_parameter<int>("min_cluster_points", 5);
    detect_threshold_m_  = this->declare_parameter<double>("detect_threshold_m", 6.0);
    enable_gui_          = this->declare_parameter<bool>("enable_gui", true);

    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&ObjectDetectionNode::onScan, this, _1));

    sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/resized_image", 10,
        std::bind(&ObjectDetectionNode::onImage, this, _1));

    pub_obj_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/object_info", 10);
    
    // --- YOLO 초기화 (경로/옵션은 네 환경에 맞게) ---
    try {
      net_ = cv::dnn::readNet("/home/helloosy/250805/2025-kookmin-contest/modular/object_detection/best.onnx");
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
      yolo_ok_ = true;
    } catch (const cv::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "YOLO load failed: %s", e.what());
      yolo_ok_ = false;
    }
    conf_threshold_ = 0.55f;
    nms_threshold_  = 0.40f;
    min_w_pix_      = 50;
    min_h_pix_      = 30;

    if (enable_gui_) {
      cv::namedWindow("OBJECT DEBUG", cv::WINDOW_AUTOSIZE);
      cv::namedWindow("CAMERA VIEW", cv::WINDOW_AUTOSIZE);
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33), std::bind(&ObjectDetectionNode::onTimer, this));
    }
    // ✅ 퍼블리시 전용 타이머 추가 (예: 30Hz)
    pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&ObjectDetectionNode::onPublishTick, this));
  }

  ~ObjectDetectionNode() override {
    if (enable_gui_) {
      cv::destroyWindow("OBJECT DEBUG");
      cv::destroyWindow("CAMERA VIEW");
    }
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const int N = static_cast<int>(msg->ranges.size());
    if (N == 0 || msg->angle_increment <= 0.0) {
      publishEmpty();
      return;
    }

    const double fov_rad = front_fov_deg_ * M_PI / 180.0;
    const double ang_lo = -fov_rad;
    const double ang_hi = +fov_rad;

    auto angleToIndex = [&](double angle) -> int {
      int idx = static_cast<int>(std::round((angle - msg->angle_min) / msg->angle_increment));
      if (idx < 0 || idx >= N) return -1;
      return idx;
    };

    int i_lo = angleToIndex(ang_lo);
    int i_hi = angleToIndex(ang_hi);
    if (i_lo == -1 && i_hi == -1) { publishEmpty(); return; }
    if (i_lo == -1) i_lo = 0;
    if (i_hi == -1) i_hi = N - 1;
    if (i_lo > i_hi) std::swap(i_lo, i_hi);

    struct Pnt { int idx; float r; double ang; };
    std::vector<Pnt> valid;
    valid.reserve(i_hi - i_lo + 1);

    for (int i = i_lo; i <= i_hi; ++i) {
      float r = msg->ranges[i];
      if (!std::isfinite(r)) continue;
      if (r < msg->range_min || r > msg->range_max) continue;
      if (r < range_min_m_   || r > range_max_m_)   continue;
      double ang = msg->angle_min + i * msg->angle_increment;
      valid.push_back({i, r, ang});
    }
    if (valid.empty()) { publishEmpty(); return; }

    struct Cluster { int start_idx, end_idx; float min_r; double min_r_ang; int count; };
    std::vector<Cluster> clusters;
    clusters.reserve(32);

    int start = 0;
    float cur_min_r = valid[0].r;
    double cur_min_ang = valid[0].ang;
    int count = 1;

    for (size_t k = 1; k < valid.size(); ++k) {
      const float dr = std::fabs(valid[k].r - valid[k-1].r);
      const bool contiguous = (valid[k].idx == valid[k-1].idx + 1);
      if (contiguous && dr <= cluster_epsilon_m_) {
        ++count;
        if (valid[k].r < cur_min_r) {
          cur_min_r = valid[k].r;
          cur_min_ang = valid[k].ang;
        }
      } else {
        clusters.push_back({ valid[start].idx, valid[k-1].idx, cur_min_r, cur_min_ang, count });
        start = static_cast<int>(k);
        cur_min_r = valid[k].r;
        cur_min_ang = valid[k].ang;
        count = 1;
      }
    }
    clusters.push_back({ valid[start].idx, valid.back().idx, cur_min_r, cur_min_ang, count });

    bool found = false;
    Cluster best{};
    best.min_r = std::numeric_limits<float>::infinity();

    for (const auto& c : clusters) {
      if (c.count < min_cluster_points_) continue;
      if (c.min_r < best.min_r) { best = c; found = true; }
    }

    if (!found) { publishEmpty(); return; }

    const double ang_start = msg->angle_min + best.start_idx * msg->angle_increment;
    const double ang_end   = msg->angle_min + best.end_idx   * msg->angle_increment;
    const double span = std::fabs(ang_end - ang_start);
    const float exists = (best.min_r <= detect_threshold_m_) ? 1.0f : 0.0f;

    // ✅ 상태만 저장
    {
      std::lock_guard<std::mutex> lk(mtx_state_);
      lidar_valid_   = true;
      st_min_r_      = best.min_r;
      st_min_r_ang_  = static_cast<float>(best.min_r_ang);
      st_span_       = static_cast<float>(span);
      st_count_      = best.count;
    }

    // 디버그 패널 숫자 업데이트만 (선택)
    if (enable_gui_) {
      std::lock_guard<std::mutex> lk(mtx_);
      dbg_dist_   = best.min_r;
      dbg_csize_  = static_cast<float>(best.count);
      last_rx_ok_ = true;
      last_rx_time_ = now();
    }
  }

  void resetLidarState() {
    std::lock_guard<std::mutex> lk(mtx_state_);
    lidar_valid_   = false;
    st_min_r_      = std::numeric_limits<float>::infinity();
    st_min_r_ang_  = 0.0f;
    st_span_       = 0.0f;
    st_count_      = 0;
  }

  void publishEmpty() {
    resetLidarState();  // ✅ 상태만 초기화

    if (enable_gui_) {
      std::lock_guard<std::mutex> lk(mtx_);
      dbg_exists_ = 0.0f;
      dbg_dist_   = std::numeric_limits<float>::infinity();
      dbg_csize_  = 0.0f;
      last_rx_ok_ = false;
    }
  }

  void onPublishTick() {
    // 스냅샷 취득
    bool  lidar_ok;
    float minr, ang, spn;
    int   cnt;
    float box_area;

    {
      std::lock_guard<std::mutex> lk(mtx_state_);
      lidar_ok = lidar_valid_;
      minr     = st_min_r_;
      ang      = st_min_r_ang_;
      spn      = st_span_;
      cnt      = st_count_;
    }
    {
      std::lock_guard<std::mutex> lk(mtx_box_);
      box_area = last_box_area_pix_;  // 박스 없으면 0
    }

    const float exists = (lidar_ok && (minr <= detect_threshold_m_)) ? 1.0f : 0.0f;

    std_msgs::msg::Float32MultiArray out;
    // [exists, min_dist, angle, span, cluster_size, box_size]
    out.data = { exists, minr, ang, spn, static_cast<float>(cnt), box_area };
    pub_obj_->publish(out);

    // 디버그 패널 숫자 최신화 (선택)
    if (enable_gui_) {
      std::lock_guard<std::mutex> lk(mtx_);
      dbg_exists_ = exists;
      dbg_dist_   = minr;
      dbg_csize_  = static_cast<float>(cnt);
      last_rx_ok_ = lidar_ok;
      last_rx_time_ = now();
    }
  }


  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat img;
    if (!enable_gui_) return;
    try {
      img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (img.empty()) return;

    if (enable_gui_) {
      std::lock_guard<std::mutex> lk(mtx_img_);
      last_img_ = img.clone();
    }

    float box_area = 0.0f; // 기본 0 (박스 없음)

    if (yolo_ok_) {
      const int W = img.cols, H = img.rows;

      cv::Mat resized;
      cv::resize(img, resized, cv::Size(640, 640));
      cv::Mat blob;
      cv::dnn::blobFromImage(resized, blob, 1.0/255.0, cv::Size(), cv::Scalar(), true, false);
      net_.setInput(blob);

      cv::Mat out;
      try {
        out = net_.forward();
        // (8400,5) 형태로 변환: (cx,cy,w,h,conf) 가정
        out = out.reshape(1, {5, 8400});
        out = out.t();
      } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "YOLO forward error: %s", e.what());
        out.release();
      }

      if (!out.empty()) {
        std::vector<cv::Rect> boxes;
        std::vector<float> confs;
        boxes.reserve(64); confs.reserve(64);

        for (int i = 0; i < out.rows; ++i) {
          float* d = out.ptr<float>(i);
          float cx = d[0], cy = d[1], w = d[2], h = d[3], conf = d[4];
          if (conf < conf_threshold_) continue;

          // 640 → 원본 스케일 복원
          float x = (cx - w/2.f) * (static_cast<float>(W) / 640.f);
          float y = (cy - h/2.f) * (static_cast<float>(H) / 640.f);
          float ww = w * (static_cast<float>(W) / 640.f);
          float hh = h * (static_cast<float>(H) / 640.f);

          int left = static_cast<int>(std::round(x));
          int top  = static_cast<int>(std::round(y));
          int wi   = static_cast<int>(std::round(ww));
          int hi   = static_cast<int>(std::round(hh));
          if (wi < min_w_pix_ || hi < min_h_pix_) continue;

          // 클리핑
          left = std::max(0, std::min(left, W-1));
          top  = std::max(0, std::min(top , H-1));
          wi   = std::max(1, std::min(wi, W - left));
          hi   = std::max(1, std::min(hi, H - top));

          boxes.emplace_back(left, top, wi, hi);
          confs.push_back(conf);
        }

        if (!boxes.empty()) {
          std::vector<int> keep;
          cv::dnn::NMSBoxes(boxes, confs, conf_threshold_, nms_threshold_, keep);
          if (!keep.empty()) {
            int best = keep[0];
            for (int idx : keep) if (confs[idx] > confs[best]) best = idx;
            const cv::Rect& b = boxes[best];
            box_area = static_cast<float>(b.width) * static_cast<float>(b.height); // 픽셀 넓이

            // (옵션) GUI에 박스 그리기
            if (enable_gui_) {
              cv::Mat overlay;
              {
                std::lock_guard<std::mutex> lk(mtx_img_);
                if (!last_img_.empty()) overlay = last_img_;
              }
              if (!overlay.empty()) {
                cv::rectangle(overlay, b, cv::Scalar(0,255,0), 2);
              }
            }
          }
        }
      }
    }

    // 공유 변수 갱신
    {
      std::lock_guard<std::mutex> lk(mtx_box_);
      last_box_area_pix_ = box_area;   // 박스 없으면 0
    }
  }

  void onTimer() {
    // OBJECT DEBUG
    {
      cv::Mat canvas(240, 480, CV_8UC3, cv::Scalar(30,30,30));
      float exists, dist, csz;
      bool ok;
      rclcpp::Time t_last;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        exists = dbg_exists_; dist = dbg_dist_; csz = dbg_csize_;
        ok = last_rx_ok_; t_last = last_rx_time_;
      }
      auto fmt = [](float v,int p=2){std::ostringstream o;o.setf(std::ios::fixed);o<<std::setprecision(p)<<v;return o.str();};
      std::string l1 = "exists: " + std::string((exists>=0.5f)?"1":"0");
      std::string l2 = "distance[m]: " + (std::isfinite(dist)?fmt(dist,2):"inf");
      std::string l3 = "cluster_size: " + fmt(csz,0);

      cv::putText(canvas, l1, {20,80}, cv::FONT_HERSHEY_SIMPLEX, 0.9,
                  (exists>=0.5f?cv::Scalar(60,220,60):cv::Scalar(40,40,200)),2);
      cv::putText(canvas, l2, {20,130}, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(230,230,230),2);
      cv::putText(canvas, l3, {20,180}, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(230,230,230),2);
      cv::imshow("OBJECT DEBUG", canvas);
    }

    // CAMERA VIEW
    {
      std::lock_guard<std::mutex> lk(mtx_img_);
      if (!last_img_.empty()) {
        cv::imshow("CAMERA VIEW", last_img_);
      }
    }

    cv::waitKey(1);
  }

  // 파라미터
  double front_fov_deg_, range_min_m_, range_max_m_, cluster_epsilon_m_, detect_threshold_m_;
  int    min_cluster_points_;
  bool   enable_gui_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_obj_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  // Debug 상태
  std::mutex mtx_;
  float dbg_exists_ = 0.0f;
  float dbg_dist_   = std::numeric_limits<float>::infinity();
  float dbg_csize_  = 0.0f;
  bool  last_rx_ok_ = false;
  rclcpp::Time last_rx_time_{0,0,RCL_ROS_TIME};

  std::mutex mtx_img_;
  cv::Mat last_img_;

  // --- YOLO 상태 ---
  cv::dnn::Net net_;
  bool  yolo_ok_ = false;
  float conf_threshold_, nms_threshold_;
  int   min_w_pix_, min_h_pix_;

  // --- YOLO 박스 면적 공유 변수 ---
  std::mutex mtx_box_;
  float last_box_area_pix_ = 0.0f;   // 박스 없으면 0

  // 퍼블리시용 공유 상태
  std::mutex mtx_state_;
  bool  lidar_valid_ = false;
  float st_min_r_     = std::numeric_limits<float>::infinity();
  float st_min_r_ang_ = 0.0f;
  float st_span_      = 0.0f;
  int   st_count_     = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
