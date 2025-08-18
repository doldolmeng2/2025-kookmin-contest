#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>
#include <functional>  // std::bind, std::placeholders::_1
#include <chrono>

using std::placeholders::_1;

class LidarViewer : public rclcpp::Node {
public:
  LidarViewer()
  : Node("rubbercone"),
    window_size_(800),
    scale_(500.0f),
    OFFSET_GAIN_(300.0f),
    rubber_offset_value_(0),
    rubber_end_value_(0)
  {
    // QoS는 LaserScan에 맞춰 SensorDataQoS 사용
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&LidarViewer::scanCallback, this, _1));

    info_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      "rubbercone_info", 10);

    info_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&LidarViewer::publishInfo, this));
  }

private:
  void publishInfo() {
    std_msgs::msg::Int32MultiArray msg;
    msg.data.resize(2);
    msg.data[0] = rubber_offset_value_;
    msg.data[1] = rubber_end_value_;
    info_pub_->publish(msg);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // 1) 유효 포인트 수집
    const float ANG_MAX = 85.0f * static_cast<float>(M_PI) / 180.0f;
    std::vector<cv::Point2f> pts;
    float angle = msg->angle_min;

    pts.reserve(msg->ranges.size());
    for (float range : msg->ranges) {
      if (std::isfinite(range) &&
          range >= 0.18f && range <= 1.90f &&
          angle >= -ANG_MAX && angle <= ANG_MAX) {
        // 좌표계: x=전방, y=좌측(표준적 레이저 프레임 가정)
        pts.emplace_back(range * std::cos(angle),
                         range * std::sin(angle));
      }
      angle += msg->angle_increment;
    }

    if (pts.size() < 2) {
      rubber_end_value_ = 0;
      return;
    }

    // 2) 라바콘 그룹화 (왼쪽/오른쪽)
    std::vector<cv::Point2f> left_group, right_group;
    cv::Point2f left_first{}, right_first{};
    float left_min = 1e6f, right_min = 1e6f;
    bool found_left = false, found_right = false;

    for (const auto &pt : pts) {
      float d = std::hypot(pt.x, pt.y);
      if (pt.y > 0) {  // 왼쪽
        if (d < left_min) { left_min = d; left_first = pt; found_left = true; }
      } else {         // 오른쪽(<=0)
        if (d < right_min) { right_min = d; right_first = pt; found_right = true; }
      }
    }

    const float CONE_D = 0.41f;  // 추정 콘 간격(미터)
    int start = 0;  // 시작 플래그 (0: 출발 전, 1: 출발 후)

    auto extend_group = [&](bool left_side,
                            std::vector<cv::Point2f>& group,
                            const cv::Point2f& first) {
      if (!std::isfinite(first.x) || !std::isfinite(first.y)) return;
      group.clear();
      group.push_back(first);
      cv::Point2f cur = first;

      // 최대 5개까지 확장
      while (group.size() < 5) {
        cv::Point2f next{};
        float best = 1e6f;
        bool ok = false;

        for (const auto &pt : pts) {
          if (left_side && pt.y <= 0) continue;
          if (!left_side && pt.y >= 0) continue;

          float d0 = std::hypot(pt.x, pt.y);
          float dc = std::hypot(pt.x - cur.x, pt.y - cur.y);

          if (d0 > std::hypot(cur.x, cur.y) + 0.10f &&
              std::fabs(dc - CONE_D) < 0.10f && d0 < best) {

            bool close = false;
            for (const auto &ex : group) {
              if (std::hypot(pt.x - ex.x, pt.y - ex.y) < 0.10f) {
                close = true;
                break;
              }
            }
            if (!close) { best = d0; next = pt; ok = true; }
          }
        }
        if (!ok) break;
        group.push_back(next);
        cur = next;
      }
    };

    if (found_left)  extend_group(true,  left_group,  left_first);
    if (found_right) extend_group(false, right_group, right_first);

    // 3) 목표점 계산 및 offset 업데이트
    bool has_mid = false;
    cv::Point2f target{0.f, 0.f};

    if (left_group.size() >= 2 && right_group.size() >= 2) {
      // 케이스 1: 양쪽 2개 이상
      cv::Point2f lm{ (left_group[0].x + left_group[1].x) * 0.5f,
                      (left_group[0].y + left_group[1].y) * 0.5f };
      cv::Point2f rm{ (right_group[0].x + right_group[1].x) * 0.5f,
                      (right_group[0].y + right_group[1].y) * 0.5f };
      target = cv::Point2f{ (lm.x + rm.x) * 0.5f, (lm.y + rm.y) * 0.5f };
      has_mid = true;

    } else if (left_group.size() == 1 && right_group.size() >= 2) {
      // 케이스 2: 왼쪽 1개, 오른쪽 2개 이상
      cv::Point2f rm{ (right_group[0].x + right_group[1].x) * 0.5f,
                      (right_group[0].y + right_group[1].y) * 0.5f };
      target = cv::Point2f{ (left_group[0].x + rm.x) * 0.5f,
                            (left_group[0].y + rm.y) * 0.5f };
      has_mid = true;

    } else if (left_group.size() >= 2 && right_group.size() == 1) {
      // 케이스 3: 왼쪽 2개 이상, 오른쪽 1개
      cv::Point2f lm{ (left_group[0].x + left_group[1].x) * 0.5f,
                      (left_group[0].y + left_group[1].y) * 0.5f };
      target = cv::Point2f{ (lm.x + right_group[0].x) * 0.5f,
                            (lm.y + right_group[0].y) * 0.5f };
      has_mid = true;

    } else if (left_group.empty() && right_group.size() >= 2) {
      // 케이스 4: 오른쪽 2개 이상만 보일 때 → 법선방향으로 차선 중앙 추정
      start = 1;
      const cv::Point2f &R0 = right_group[0];
      const cv::Point2f &R1 = right_group[1];
      cv::Point2f mid{ (R0.x + R1.x) * 0.5f, (R0.y + R1.y) * 0.5f };
      cv::Point2f v{ R1.x - R0.x, R1.y - R0.y };
      float norm = std::hypot(v.x, v.y);
      RCLCPP_DEBUG(get_logger(),
                   "Right group: (%.3f, %.3f) -> (%.3f, %.3f), norm=%.3f",
                   R0.x, R0.y, R1.x, R1.y, norm);

      if (norm > 1e-6f) {
        cv::Point2f u{ -v.y / norm, v.x / norm }; // v의 좌측 법선
        float d = CONE_D; // 차선 폭을 CONE_D로 가정(필요시 파라미터화)
        target = cv::Point2f{ mid.x + u.x * d, mid.y + u.y * d };
        has_mid = true;
      }
    }

    if (has_mid) {
      float offset = -target.y * OFFSET_GAIN_;
      rubber_offset_value_ = static_cast<int32_t>(std::round(offset));
      rubber_end_value_ = 0;

      RCLCPP_DEBUG(get_logger(),
                   "Target(%.3f, %.3f) → offset=%d",
                   target.x, target.y, rubber_offset_value_);
    } else if(start == 1) {
      // 출발이후 중앙 추정 불가 → 오프셋 하드코딩 -50, 0.4s 동안 유지
      if (!hardcode_active_) {
        hardcode_active_ = true;
        hardcode_start_time_ = this->now();
        rubber_offset_value_ = -50;
        rubber_end_value_ = 0;  // 아직 종료 아님
        RCLCPP_INFO(get_logger(), "Hardcoded offset -50 started");
      } else {
        // 이미 하드코딩 주행 중 → 경과시간 확인
        const double elapsed = (this->now() - hardcode_start_time_).seconds();
        if (elapsed < 1.1) {  // 400ms
          rubber_offset_value_ = -50;
          rubber_end_value_ = 0;
        } else {
          hardcode_active_ = false;
          rubber_offset_value_ = 0;
          rubber_end_value_ = 1;  // 최종 종료
          RCLCPP_INFO(get_logger(), "Hardcoded offset finished, end=1");
        }
      }
    }
  } // <-- scanCallback 끝


private:
  // params / states
  int   window_size_;
  float scale_;
  const float OFFSET_GAIN_;

  int32_t rubber_offset_value_;
  int32_t rubber_end_value_;
  
  //하드코어 오프셋제어용
  rclcpp::Time hardcode_start_time_;
  bool hardcode_active_ = false;

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr info_timer_;
  
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LidarViewer>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("rubbercone_main"),
                 "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
