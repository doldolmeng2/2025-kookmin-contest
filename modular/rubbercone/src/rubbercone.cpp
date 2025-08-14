#include "rubbercone/rubbercone.hpp"
#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <cstdlib>
#include <chrono>
#include <opencv2/opencv.hpp>

LidarViewer::LidarViewer()
: Node("rubbercone"),
  window_size_(800),        
  scale_(500.0f),
  OFFSET_GAIN_(300.0f),
  rubber_offset_value_(0),
  rubber_end_value_(0)
{
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&LidarViewer::scanCallback, this, std::placeholders::_1)
  );
  info_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
    "rubbercone_info", 10
  );
  info_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&LidarViewer::publishInfo, this)
  );
}

void LidarViewer::publishInfo() {
  std_msgs::msg::Int32MultiArray msg;
  msg.data.resize(2);
  msg.data[0] = rubber_offset_value_;
  msg.data[1] = rubber_end_value_;
  info_pub_->publish(msg);
}

void LidarViewer::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // 1) 유효 포인트 수집
  const float ANG_MAX = 85.0f * M_PI / 180.0f;
  std::vector<cv::Point2f> pts;
  float angle = msg->angle_min;
  
  for (float range : msg->ranges) {
    if (std::isfinite(range) &&
        range >= 0.18f && range <= 1.90f &&
        angle >= -ANG_MAX && angle <= ANG_MAX) {
      pts.emplace_back(range * std::cos(angle),
                       range * std::sin(angle));
    }
    angle += msg->angle_increment;
  }

  // 포인트 수가 부족한 경우 처리
  if (pts.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Insufficient valid points: %zu", pts.size());
    rubber_offset_value_ = 0;
    return;
  }

  // 2) 라바콘 그룹화 (왼쪽/오른쪽)
  std::vector<cv::Point2f> left_group, right_group;
  cv::Point2f left_first, right_first;
  float left_min = 1e6f, right_min = 1e6f;
  bool found_left = false, found_right = false;

  for (auto &pt : pts) {
    float d = std::hypot(pt.x, pt.y);
    if (pt.y > 0) {  // 왼쪽 (y > 0)
      if (d < left_min) {
        left_min = d; 
        left_first = pt; 
        found_left = true;
      }
    } else {  // 오른쪽 (y <= 0)
      if (d < right_min) {
        right_min = d; 
        right_first = pt; 
        found_right = true;
      }
    }
  }

  const float CONE_D = 0.38f;
  
  // 왼쪽 그룹 확장
  if (found_left) {
    left_group.push_back(left_first);
    cv::Point2f cur = left_first;
    
    while (left_group.size() < 5) {
      cv::Point2f next; 
      float best = 1e6f; 
      bool ok = false;
      
      for (auto &pt : pts) {
        if (pt.y <= 0) continue;  // 왼쪽만
        
        float d0 = std::hypot(pt.x, pt.y);
        float dc = std::hypot(pt.x - cur.x, pt.y - cur.y);
        
        if (d0 > std::hypot(cur.x, cur.y) + 0.1f &&
            std::abs(dc - CONE_D) < 0.15f && d0 < best) {
          
          bool close = false;
          for (auto &ex : left_group) {
            if (std::hypot(pt.x - ex.x, pt.y - ex.y) < 0.1f) {
              close = true;
              break;
            }
          }
          
          if (!close) { 
            best = d0; 
            next = pt; 
            ok = true; 
          }
        }
      }
      
      if (!ok) break;
      left_group.push_back(next);
      cur = next;
    }
  }
  
  // 오른쪽 그룹 확장
  if (found_right) {
    right_group.push_back(right_first);
    cv::Point2f cur = right_first;
    
    while (right_group.size() < 5) {
      cv::Point2f next; 
      float best = 1e6f; 
      bool ok = false;
      
      for (auto &pt : pts) {
        if (pt.y >= 0) continue;  // 오른쪽만
        
        float d0 = std::hypot(pt.x, pt.y);
        float dc = std::hypot(pt.x - cur.x, pt.y - cur.y);
        
        if (d0 > std::hypot(cur.x, cur.y) + 0.1f &&
            std::abs(dc - CONE_D) < 0.15f && d0 < best) {
          
          bool close = false;
          for (auto &ex : right_group) {
            if (std::hypot(pt.x - ex.x, pt.y - ex.y) < 0.1f) {
              close = true;
              break;
            }
          }
          
          if (!close) { 
            best = d0; 
            next = pt; 
            ok = true; 
          }
        }
      }
      
      if (!ok) break;
      right_group.push_back(next);
      cur = next;
    }
  }

  // 3) 목표점 계산 및 offset 업데이트
  bool has_mid = false;
  cv::Point2f target;
  
  if (left_group.size() >= 2 && right_group.size() >= 2) {
    // 케이스 1: 양쪽 모두 2개 이상의 콘이 있는 경우
    cv::Point2f lm{(left_group[0].x + left_group[1].x) * 0.5f,
                   (left_group[0].y + left_group[1].y) * 0.5f};
    cv::Point2f rm{(right_group[0].x + right_group[1].x) * 0.5f,
                   (right_group[0].y + right_group[1].y) * 0.5f};
    target = cv::Point2f{(lm.x + rm.x) / 2.0f, (lm.y + rm.y) * 0.5f};
    has_mid = true;
    
  } else if (left_group.size() == 1 && right_group.size() >= 2) {
    // 케이스 2: 왼쪽 1개, 오른쪽 2개 이상
    cv::Point2f rm{(right_group[0].x + right_group[1].x) / 2.0f,
                   (right_group[0].y + right_group[1].y) / 2.0f};
    target = cv::Point2f{(left_group[0].x + rm.x) / 2.0f,
                         (left_group[0].y + rm.y) / 2.0f};
    has_mid = true;
    
  } else if (left_group.size() >= 2 && right_group.size() == 1) {
    // 케이스 3: 왼쪽 2개 이상, 오른쪽 1개
    cv::Point2f lm{(left_group[0].x + left_group[1].x) / 2.0f,
                   (left_group[0].y + left_group[1].y) / 2.0f};
    target = cv::Point2f{(lm.x + right_group[0].x) / 2.0f,
                         (lm.y + right_group[0].y) / 2.0f};
    has_mid = true;
    
  } else if (left_group.empty() && right_group.size() >= 2) {
    // 케이스 4: 왼쪽 없음, 오른쪽 2개 이상
    const cv::Point2f &R0 = right_group[0];
    const cv::Point2f &R1 = right_group[1];
    cv::Point2f mid{(R0.x + R1.x) * 0.5f, (R0.y + R1.y) * 0.5f};
    cv::Point2f v{R1.x - R0.x, R1.y - R0.y};
    float norm = std::hypot(v.x, v.y);
    
    if (norm > 1e-6f) {  // 0으로 나누기 방지
      cv::Point2f u{-v.y / norm, v.x / norm};
      float d = 0.37f;
      target = cv::Point2f{mid.x + u.x * d, mid.y + u.y * d};
      has_mid = true;
    }
    
  } else if (left_group.size() >= 2 && right_group.empty()) {
    // 케이스 5: 왼쪽 2개 이상, 오른쪽 없음
    const cv::Point2f &L0 = left_group[0];
    const cv::Point2f &L1 = left_group[1];
    cv::Point2f mid{(L0.x + L1.x) * 0.5f, (L0.y + L1.y) * 0.5f};
    cv::Point2f v{L1.x - L0.x, L1.y - L0.y};
    float norm = std::hypot(v.x, v.y);
    
    if (norm > 1e-6f) {  // 0으로 나누기 방지
      cv::Point2f u{v.y / norm, -v.x / norm};  // 반대 방향
      float d = 0.37f;
      target = cv::Point2f{mid.x + u.x * d, mid.y + u.y * d};
      has_mid = true;
    }
  }

  // offset 값 업데이트
  if (has_mid) {
    float offset = -target.y * OFFSET_GAIN_;
    rubber_offset_value_ = static_cast<int32_t>(offset);
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "Target: (%.3f, %.3f), Offset: %d", 
                 target.x, target.y, rubber_offset_value_);
  } else {
    rubber_offset_value_ = 0;
    
    RCLCPP_INFO(this->get_logger(),
                "rubber_offset_value_ = 0, lane_detection_ = 1 (L:%zu, R:%zu)",
                left_group.size(), right_group.size());
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  try {
    rclcpp::spin(std::make_shared<LidarViewer>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception caught: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
