#include "rubbercone/rubbercone.hpp"
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <cstdlib>

LidarViewer::LidarViewer()
: Node("rubbercone"),
  window_size_(800),
  scale_(500.0f),
  OFFSET_GAIN_(300.0f),
  rubber_offset_value_(0),
  rubber_end_value_(0),
  center_(window_size_/2, window_size_/2)
{
  createBackground();
  valid_points_.reserve(500);
  left_indices_.reserve(50);
  right_indices_.reserve(50);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&LidarViewer::fastScanCallback, this, std::placeholders::_1)
  );

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&LidarViewer::imageCallback, this, std::placeholders::_1)
  );

  info_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
    "rubbercone_info", 10
  );

  info_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&LidarViewer::publishInfo, this)
  );

  cv::namedWindow("Lidar View", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Camera View", cv::WINDOW_AUTOSIZE);
}

void LidarViewer::createBackground() {
  background_canvas_ = cv::Mat::zeros(window_size_, window_size_, CV_8UC3);
  for (int r = 10; r <= 100; r += 10) {
    int radius = int(r * (scale_/100.0f));
    cv::circle(background_canvas_, center_, radius, cv::Scalar(100,100,100), 1);
  }
}

void LidarViewer::fastScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr& msg) {
  cv::Mat canvas = background_canvas_.clone();
  valid_points_.clear();
  left_indices_.clear();
  right_indices_.clear();

  const float* ranges = msg->ranges.data();
  const size_t n = msg->ranges.size();
  const float a0 = msg->angle_min;
  const float da = msg->angle_increment;
  const float max_ang = 85.0f * M_PI / 180.0f;

  for (size_t i = 0; i < n; ++i) {
    float r = ranges[i];
    float ang = a0 + i * da;
    if (r >= 0.18f && r <= 1.20f && ang >= -max_ang && ang <= max_ang) {
      float x = r * std::cos(ang);
      float y = r * std::sin(ang);
      valid_points_.emplace_back(x, y);
      int idx = valid_points_.size() - 1;
      (y > 0 ? left_indices_ : right_indices_).push_back(idx);
    }
  }

  auto L = detectCones(left_indices_, !after_cone_mode_);
  auto R = detectCones(right_indices_, !after_cone_mode_);

  float tx = 0, ty = 0;
  bool ok = computeTarget(L, R, tx, ty);
  visualize(canvas, L, R, tx, ty, ok);

  if (!after_cone_mode_ && left_indices_.empty()) {
    after_cone_mode_ = true;
  }

  if (after_cone_mode_ && right_indices_.size() >= 2) {
    const auto& P = valid_points_[ right_indices_[0] ];
    const auto& Q = valid_points_[ right_indices_[1] ];
    cv::Point2f mid{ (P.first+Q.first)*0.5f, (P.second+Q.second)*0.5f };
    cv::Point2f v{ Q.first-P.first, Q.second-P.second };
    float inv = 1.0f/std::hypot(v.x, v.y);
    cv::Point2f perp{ -v.y*inv, v.x*inv };
    constexpr float OFFSET_M = 0.37f;
    lane_target_ = { mid.x+perp.x*OFFSET_M, mid.y+perp.y*OFFSET_M };
    lane_target_ready_ = true;
    int lx = int(center_.x - lane_target_.y * scale_);
    int ly = int(center_.y - lane_target_.x * scale_);
    cv::circle(canvas, {lx, ly}, 6, cv::Scalar(0,255,255), -1);
    cv::putText(canvas, "LaneTarget", {lx+5, ly-5},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1);
  }

  if (left_indices_.size() <= 1 && right_indices_.size() <= 1) {
    slow_mode_     = true;
    current_speed_ = base_speed_ * 0.2f;
  } else if (left_indices_.size() >= 2 && right_indices_.size() >= 2) {
    slow_mode_     = false;
    current_speed_ = base_speed_;
  }

  cv::imshow("Lidar View", canvas);
  cv::waitKey(1);
}

std::vector<int> LidarViewer::detectCones(const std::vector<int>& idxs, bool use_base_tol) {
  float tol_sq = use_base_tol ? BASE_TOLERANCE_SQ : AFTER_TOLERANCE_SQ;
  std::vector<int> cones;
  if (idxs.empty()) return cones;
  cones.reserve(5);
  int best = *std::min_element(idxs.begin(), idxs.end(),
    [&](int a,int b) {
      auto &A = valid_points_[a], &B = valid_points_[b];
      return A.first*A.first + A.second*A.second
           < B.first*B.first + B.second*B.second;
    }
  );
  cones.push_back(best);

  while (cones.size() < 5) {
    auto &c = valid_points_[cones.back()];
    float cd = c.first*c.first + c.second*c.second;
    int ni = -1;
    float bd = 1e9f;
    for (int i : idxs) {
      if (std::find(cones.begin(), cones.end(), i) != cones.end()) continue;
      auto &p = valid_points_[i];
      float d0 = p.first*p.first + p.second*p.second;
      if (d0 <= cd + 0.01f) continue;
      float dx = p.first - c.first, dy = p.second - c.second;
      float d1 = dx*dx + dy*dy;
      if (std::abs(d1 - CONE_DIST_SQ) < tol_sq && d0 < bd) {
        bd = d0; ni = i;
      }
    }
    if (ni < 0) break;
    cones.push_back(ni);
  }
  return cones;
}

bool LidarViewer::computeTarget(const std::vector<int>& L,
                                const std::vector<int>& R,
                                float& tx, float& ty) {
  auto mid = [&](auto &C, int cnt) {
    if (cnt == 1) return valid_points_[C[0]];
    float sx = 0, sy = 0;
    for (int i = 0; i < cnt; ++i) {
      auto &p = valid_points_[C[i]];
      sx += p.first; sy += p.second;
    }
    return std::pair<float,float>{sx/cnt, sy/cnt};
  };

  if (L.size() >= 2 && R.size() >= 2) {
    auto a = mid(L,2), b = mid(R,2);
    tx = (a.first + b.first) * 0.5f;
    ty = (a.second + b.second) * 0.5f;
  } else if (L.size() == 1 && R.size() >= 2) {
    auto a = mid(L,1), b = mid(R,2);
    tx = (a.first + b.first) * 0.5f;
    ty = (a.second + b.second) * 0.5f;
  } else if (R.size() == 1 && L.size() >= 2) {
    auto a = mid(L,2), b = mid(R,1);
    tx = (a.first + b.first) * 0.5f;
    ty = (a.second + b.second) * 0.5f;
  } else {
    return false;
  }
  return true;
}

void LidarViewer::visualize(cv::Mat& C,
                            const std::vector<int>& L,
                            const std::vector<int>& R,
                            float tx, float ty, bool ok) {
  for (int i : L) {
    auto &p = valid_points_[i];
    cv::circle(C, {int(center_.x - p.second*scale_),
                   int(center_.y - p.first*scale_)},
               4, cv::Scalar(255,0,0), -1);
  }
  for (int i : R) {
    auto &p = valid_points_[i];
    cv::circle(C, {int(center_.x - p.second*scale_),
                   int(center_.y - p.first*scale_)},
               4, cv::Scalar(0,0,255), -1);
  }
  if (ok) {
    cv::circle(C, {int(center_.x - ty*scale_),
                   int(center_.y - tx*scale_)},
               8, cv::Scalar(0,255,0), -1);
    rubber_offset_value_ = int(-ty * OFFSET_GAIN_);
    cv::putText(C, "Offset:" + std::to_string(rubber_offset_value_),
                {10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 1);
  } else {
    rubber_offset_value_ = 0;
    cv::putText(C, "No Target", {10,30},
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 1);
  }
}

void LidarViewer::publishInfo() {
  std_msgs::msg::Int32MultiArray msg;
  msg.data.resize(6);
  msg.data[0] = rubber_offset_value_;
  msg.data[1] = rubber_end_value_;
  msg.data[2] = slow_mode_ ? int(current_speed_*100) : int(base_speed_*100);
  msg.data[3] = lane_target_ready_ ? int(lane_target_.x*100) : 0;
  msg.data[4] = lane_target_ready_ ? int(lane_target_.y*100) : 0;
  msg.data[5] = after_cone_mode_ ? 1 : 0;
  info_pub_->publish(msg);
}

void LidarViewer::imageCallback(const sensor_msgs::msg::Image::SharedPtr& msg) {
  cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
  cv::imshow("Camera View", img);
  cv::waitKey(1);
}

int main(int argc, char **argv) {
  setenv("GDK_BACKEND", "x11", 1);
  setenv("QT_QPA_PLATFORM", "xcb", 1);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarViewer>());
  rclcpp::shutdown();
  return 0;
}
