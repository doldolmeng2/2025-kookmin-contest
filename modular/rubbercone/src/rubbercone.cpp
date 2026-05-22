// ─────────────────────────────────────────────────────────────────────────────
// rubbercone.cpp
//
// 역할: LiDAR 데이터를 이용한 라바콘 구간 주행 오프셋 계산 노드
//
// 알고리즘:
//   1) 전방 ±90°, 거리 0.18~1.00m 범위의 LiDAR 포인트 수집
//      (전방 ±13° 영역은 차체 내부 반사 방지를 위해 제외)
//   2) 왼쪽(y>0)과 오른쪽(y≤0)으로 나눠 가장 가까운 콘을 시작점으로 설정
//   3) 콘 간격(CONE_D ≈ 0.42m)을 기준으로 최대 3개까지 그룹 확장
//   4) 양쪽 그룹의 대표점 중간값을 목표점으로 계산하고
//      y 좌표에 OFFSET_GAIN_을 곱하여 조향 오프셋 산출
//   5) 양쪽 포인트가 부족하면 라바콘 구간 종료(end_flag=1)로 판단
//
// 구독: /scan (sensor_msgs/LaserScan)
// 발행: rubbercone_info (std_msgs/Int32MultiArray, [offset, end_flag])
//   offset   : 조향 오프셋 (양수=오른쪽 편향)
//   end_flag : 0=정상 주행, 1=라바콘 구간 종료
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <functional>
#include <chrono>

using std::placeholders::_1;


class LidarViewer : public rclcpp::Node {
public:
    LidarViewer()
    : Node("rubbercone"),
      OFFSET_GAIN_(300.0f),
      rubber_offset_value_(0),
      rubber_end_value_(0)
    {
        // LiDAR 스캔 구독 (SensorDataQoS: Best Effort, 최신성 우선)
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&LidarViewer::scanCallback, this, _1));

        // 오프셋/종료 플래그 발행 (20ms 주기 = 50Hz)
        auto qos_fast = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        info_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("rubbercone_info", qos_fast);

        info_timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&LidarViewer::publishInfo, this));
    }

private:
    // 최신 offset / end_flag를 주기적으로 발행
    void publishInfo() {
        std_msgs::msg::Int32MultiArray msg;
        msg.data.resize(2);
        msg.data[0] = rubber_offset_value_;  // 조향 오프셋
        msg.data[1] = rubber_end_value_;     // 종료 플래그
        info_pub_->publish(msg);
    }

    // ─────────────────────────────────────────────────────────────────────
    // LiDAR 스캔 콜백
    //
    // 스캔 데이터에서 라바콘 위치를 추정하고 오프셋을 계산한다.
    // ─────────────────────────────────────────────────────────────────────
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        const float ANG_MAX    = 90.0f  * M_PI / 180.0f;  // 탐색 최대 각도: ±90°
        const float ANG_IGNORE = 13.0f  * M_PI / 180.0f;  // 차체 전방 무시 각도: ±13°

        // 1) 유효 포인트 수집: 거리/각도 조건 필터링, 전방 영역 제외
        std::vector<cv::Point2f> pts;
        float angle = msg->angle_min;
        for (float range : msg->ranges) {
            if (std::isfinite(range) &&
                range >= 0.18f && range <= 1.00f &&
                angle >= -ANG_MAX && angle <= ANG_MAX)
            {
                // 전방 ±ANG_IGNORE 이내는 차체 반사 노이즈 가능성이 있어 제외
                if (std::abs(angle) < ANG_IGNORE) {
                    angle += msg->angle_increment;
                    continue;
                }
                pts.emplace_back(range * std::cos(angle),
                                 range * std::sin(angle));
            }
            angle += msg->angle_increment;
        }

        if (pts.size() < 2) {
            rubber_end_value_ = 0;
            return;
        }

        // 2) 왼쪽/오른쪽 그룹화: 각 방향에서 가장 가까운 포인트를 시작점으로 선택
        std::vector<cv::Point2f> left_group, right_group;
        cv::Point2f left_first{}, right_first{};
        float left_min = 1e6f, right_min = 1e6f;
        bool found_left = false, found_right = false;

        for (const auto& pt : pts) {
            float d = std::hypot(pt.x, pt.y);
            if (pt.y > 0) {  // 왼쪽
                if (d < left_min) { left_min = d; left_first = pt; found_left = true; }
            } else {          // 오른쪽
                if (d < right_min) { right_min = d; right_first = pt; found_right = true; }
            }
        }

        // 콘 간격 기준 (m): 인접 콘까지의 예상 거리
        const float CONE_D = 0.42f;

        // 시작점에서 콘 간격을 기준으로 최대 3개까지 그룹을 확장하는 람다
        auto extend_group = [&](bool left_side,
                                std::vector<cv::Point2f>& group,
                                const cv::Point2f& first)
        {
            if (!std::isfinite(first.x) || !std::isfinite(first.y)) return;
            group.clear();
            group.push_back(first);
            cv::Point2f cur = first;

            while (group.size() < 3) {
                cv::Point2f next{};
                float best = 1e6f;
                bool  ok   = false;

                for (const auto& pt : pts) {
                    if (left_side  && pt.y <= 0) continue;
                    if (!left_side && pt.y >= 0) continue;

                    float d0 = std::hypot(pt.x, pt.y);
                    float dc = std::hypot(pt.x - cur.x, pt.y - cur.y);

                    // 더 멀리 있고, 콘 간격과 유사하며, 이미 그룹에 없는 포인트
                    if (d0 > std::hypot(cur.x, cur.y) + 0.10f &&
                        std::fabs(dc - CONE_D) < 0.10f && d0 < best)
                    {
                        bool close = false;
                        for (const auto& ex : group) {
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

        // 3) 목표점 계산 및 오프셋 산출
        bool has_mid = true;
        cv::Point2f target{0.f, 0.f};

        if (left_group.size() >= 2 && right_group.size() >= 2) {
            // 케이스 1: 양쪽 2개 이상 → 각 그룹 앞 두 콘의 중간값의 중간
            if (start < 30) {
                start++;
            }
            cv::Point2f lm{ (left_group[0].x  + left_group[1].x)  * 0.5f,
                            (left_group[0].y  + left_group[1].y)  * 0.5f };
            cv::Point2f rm{ (right_group[0].x + right_group[1].x) * 0.5f,
                            (right_group[0].y + right_group[1].y) * 0.5f };
            target  = cv::Point2f{ (lm.x + rm.x) * 0.5f, (lm.y + rm.y) * 0.5f };
            has_mid = true;

        } else if (left_group.size() == 1 && right_group.size() >= 2) {
            // 케이스 2: 왼쪽 1개, 오른쪽 2개 이상
            cv::Point2f rm{ (right_group[0].x + right_group[1].x) * 0.5f,
                            (right_group[0].y + right_group[1].y) * 0.5f };
            target  = cv::Point2f{ (left_group[0].x + rm.x) * 0.5f,
                                   (left_group[0].y + rm.y) * 0.5f };
            has_mid = true;

        } else if (left_group.size() >= 2 && right_group.size() == 1) {
            // 케이스 3: 왼쪽 2개 이상, 오른쪽 1개
            cv::Point2f lm{ (left_group[0].x + left_group[1].x) * 0.5f,
                            (left_group[0].y + left_group[1].y) * 0.5f };
            target  = cv::Point2f{ (lm.x + right_group[0].x) * 0.5f,
                                   (lm.y + right_group[0].y) * 0.5f };
            has_mid = true;

        } else if (start == 30) {
            // 포인트 부족 + 안정화 구간 경과 → 라바콘 구간 종료
            has_mid = false;
        }

        if (has_mid) {
            // 목표점의 y 좌표(좌우 편향)에 OFFSET_GAIN_을 곱해 조향 오프셋 생성
            // y > 0이면 왼쪽 편향 → 음수 offset(오른쪽으로 조향)
            float offset = -target.y * OFFSET_GAIN_;
            rubber_offset_value_ = static_cast<int32_t>(std::round(offset));
            rubber_end_value_    = 0;
        } else {
            rubber_end_value_ = 1;  // 라바콘 구간 종료 신호
        }
    }

    // ── 상수 ────────────────────────────────────────────────────────────
    const float OFFSET_GAIN_;  // y 편향 → 조향 오프셋 변환 계수

    // ── 상태 변수 ───────────────────────────────────────────────────────
    int     start = 0;              // 초기 안정화 카운터 (30 프레임까지)
    int32_t rubber_offset_value_;   // 최신 조향 오프셋
    int32_t rubber_end_value_;      // 라바콘 종료 플래그 (0/1)

    // ── ROS 통신 객체 ───────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr     info_pub_;
    rclcpp::TimerBase::SharedPtr                                     info_timer_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<LidarViewer>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rubbercone_main"), "예외: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
