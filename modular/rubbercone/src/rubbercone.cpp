#include "rubbercone/rubbercone.hpp"
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <cstdlib>
#include <chrono>
#include <algorithm>

LidarViewer::LidarViewer()
: Node("rubbercone"), window_size_(800), scale_(500.0f), OFFSET_GAIN_(300.0f),
rubber_offset_value_(0),
rubber_end_value_(0)
{
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&LidarViewer::scanCallback, this, std::placeholders::_1)
    );

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&LidarViewer::imageCallback, this, std::placeholders::_1)
    );

    info_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("rubbercone_info", 10);
    
    info_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&LidarViewer::publishInfo, this)
    );

    cv::namedWindow("Lidar View", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Camera View", cv::WINDOW_AUTOSIZE);
}

void LidarViewer::publishInfo() {
  std_msgs::msg::Int32MultiArray msg;
  msg.data.resize(2);
  msg.data[0] = rubber_offset_value_;
  msg.data[1] = rubber_end_value_;
  info_pub_->publish(msg);
}

void LidarViewer::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // 1) 캔버스 준비
    cv::Mat canvas = cv::Mat::zeros(window_size_, window_size_, CV_8UC3);
    cv::Point2f center(window_size_/2, window_size_/2);

    // 2) 10cm 단위 동심원 (10~100cm)
    for (int r_cm = 10; r_cm <= 100; r_cm += 10) {
        float rad_px = r_cm * (scale_ / 100.0f);
        cv::circle(canvas, center, int(rad_px), cv::Scalar(100,100,100), 1);
        cv::putText(canvas, std::to_string(r_cm) + "cm",
                    {int(center.x + 5), int(center.y - rad_px)},
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 1);
    }

    // 3) 유효 포인트만 수집 (각도와 거리 제한) - 필터링 기준 유지
    const float ANG_MAX = 85.0f * M_PI/180.0f;
    std::vector<cv::Point2f> pts;
    float angle = msg->angle_min;
    for (float range : msg->ranges) {
        if (std::isfinite(range)
            && range >= 0.18f && range <= 1.20f
            && angle >= -ANG_MAX && angle <= ANG_MAX)
        {
            pts.emplace_back(range * std::cos(angle),
                             range * std::sin(angle));
        }
        angle += msg->angle_increment;
    }

    // 4) 새로운 그룹화 알고리즘: 순차적 라바콘 검출
    std::vector<cv::Point2f> left_group, right_group;
    
    // 좌측과 우측에서 가장 가까운 점 찾기
    cv::Point2f left_first, right_first;
    float left_min_dist = 1e6f, right_min_dist = 1e6f;
    bool found_left = false, found_right = false;
    
    for (const auto& pt : pts) {
        float dist = std::hypot(pt.x, pt.y);
        if (pt.y > 0) { // 좌측
            if (dist < left_min_dist) {
                left_min_dist = dist;
                left_first = pt;
                found_left = true;
            }
        } else { // 우측
            if (dist < right_min_dist) {
                right_min_dist = dist;
                right_first = pt;
                found_right = true;
            }
        }
    }
    
    // L그룹 구성: 첫 번째 라바콘부터 38cm 간격으로 순차 검출
    if (found_left) {
        left_group.push_back(left_first);
        cv::Point2f current = left_first;
        const float CONE_DISTANCE = 0.38f; // 38cm
        
        while (left_group.size() < 3) { // 최대 3개까지
            cv::Point2f next_cone;
            float min_valid_dist = 1e6f;
            bool found_next = false;
            
            for (const auto& pt : pts) {
                if (pt.y <= 0) continue; // 좌측만
                float dist_from_current = std::hypot(pt.x - current.x, pt.y - current.y);
                float dist_from_origin = std::hypot(pt.x, pt.y);
                
                // 현재 라바콘보다 더 멀리 있고, 38cm 근처에 있는 점 찾기
                if (dist_from_origin > std::hypot(current.x, current.y) + 0.1f && 
                    std::abs(dist_from_current - CONE_DISTANCE) < 0.15f &&
                    dist_from_origin < min_valid_dist) {
                    
                    // 이미 선택된 라바콘과 너무 가깝지 않은지 확인
                    bool too_close = false;
                    for (const auto& existing : left_group) {
                        if (std::hypot(pt.x - existing.x, pt.y - existing.y) < 0.1f) {
                            too_close = true;
                            break;
                        }
                    }
                    
                    if (!too_close) {
                        min_valid_dist = dist_from_origin;
                        next_cone = pt;
                        found_next = true;
                    }
                }
            }
            
            if (found_next) {
                left_group.push_back(next_cone);
                current = next_cone;
            } else {
                break;
            }
        }
    }
    
    // R그룹 구성: 동일한 방식
    if (found_right) {
        right_group.push_back(right_first);
        cv::Point2f current = right_first;
        const float CONE_DISTANCE = 0.38f; // 38cm
        
        while (right_group.size() < 3) { // 최대 3개까지
            cv::Point2f next_cone;
            float min_valid_dist = 1e6f;
            bool found_next = false;
            
            for (const auto& pt : pts) {
                if (pt.y >= 0) continue; // 우측만
                float dist_from_current = std::hypot(pt.x - current.x, pt.y - current.y);
                float dist_from_origin = std::hypot(pt.x, pt.y);
                
                // 현재 라바콘보다 더 멀리 있고, 38cm 근처에 있는 점 찾기
                if (dist_from_origin > std::hypot(current.x, current.y) + 0.1f && 
                    std::abs(dist_from_current - CONE_DISTANCE) < 0.15f &&
                    dist_from_origin < min_valid_dist) {
                    
                    // 이미 선택된 라바콘과 너무 가깝지 않은지 확인
                    bool too_close = false;
                    for (const auto& existing : right_group) {
                        if (std::hypot(pt.x - existing.x, pt.y - existing.y) < 0.1f) {
                            too_close = true;
                            break;
                        }
                    }
                    
                    if (!too_close) {
                        min_valid_dist = dist_from_origin;
                        next_cone = pt;
                        found_next = true;
                    }
                }
            }
            
            if (found_next) {
                right_group.push_back(next_cone);
                current = next_cone;
            } else {
                break;
            }
        }
    }

    // 5) 그룹별 라바콘을 화면에 그리기
    // L그룹을 파란색으로 표시
    for (size_t i = 0; i < left_group.size(); ++i) {
        int px = int(center.x - left_group[i].y * scale_);
        int py = int(center.y - left_group[i].x * scale_);
        cv::circle(canvas, {px, py}, 4, cv::Scalar(255,0,0), -1); // 파란색
        cv::putText(canvas, "L" + std::to_string(i+1), {px+5, py-5}, 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 1);
    }
    
    // R그룹을 빨간색으로 표시
    for (size_t i = 0; i < right_group.size(); ++i) {
        int px = int(center.x - right_group[i].y * scale_);
        int py = int(center.y - right_group[i].x * scale_);
        cv::circle(canvas, {px, py}, 4, cv::Scalar(0,0,255), -1); // 빨간색
        cv::putText(canvas, "R" + std::to_string(i+1), {px+5, py-5}, 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 1);
    }

// 6) 각 그룹의 첫 번째와 두 번째 라바콘의 중점 계산 (예외 처리 포함)
bool has_mid = false;
cv::Point2f final_target;

// 양쪽 그룹에 각각 최소 2개 이상 라바콘이 검출된 경우 (기존 로직)
if (left_group.size() >= 2 && right_group.size() >= 2) {
    // L그룹의 중점
    cv::Point2f left_mid = {
        (left_group[0].x + left_group[1].x) * 0.5f,
        (left_group[0].y + left_group[1].y) * 0.5f
    };
    // R그룹의 중점
    cv::Point2f right_mid = {
        (right_group[0].x + right_group[1].x) * 0.5f,
        (right_group[0].y + right_group[1].y) * 0.5f
    };
    // 두 중점의 중점 → 최종 목표점
    final_target = {
        (left_mid.x + right_mid.x) * 0.5f,
        (left_mid.y + right_mid.y) * 0.5f
    };
    has_mid = true;

// 예외 1) L그룹에 1개, R그룹에 2개 이상 검출된 경우
} else if (left_group.size() == 1 && right_group.size() >= 2) {
    // R그룹에서 두 개의 중점
    cv::Point2f right_mid = {
        (right_group[0].x + right_group[1].x) * 0.5f,
        (right_group[0].y + right_group[1].y) * 0.5f
    };
    // L그룹의 유일한 점
    cv::Point2f left_pt = left_group[0];
    // 두 점의 중점 → 최종 목표점
    final_target = {
        (left_pt.x + right_mid.x) * 0.5f,
        (left_pt.y + right_mid.y) * 0.5f
    };
    has_mid = true;

// 예외 2) R그룹에 1개, L그룹에 2개 이상 검출된 경우
} else if (right_group.size() == 1 && left_group.size() >= 2) {
    // L그룹에서 두 개의 중점
    cv::Point2f left_mid = {
        (left_group[0].x + left_group[1].x) * 0.5f,
        (left_group[0].y + left_group[1].y) * 0.5f
    };
    // R그룹의 유일한 점
    cv::Point2f right_pt = right_group[0];
    // 두 점의 중점 → 최종 목표점
    final_target = {
        (left_mid.x + right_pt.x) * 0.5f,
        (left_mid.y + right_pt.y) * 0.5f
    };
    has_mid = true;
}

// has_mid가 true일 때만 최종 목표점을 사용하고, 그렇지 않으면 검출 실패로 처리
if (has_mid) {
    // final_target 활용 로직...
} else {
    // 검출 실패 처리 (rubber_offset_value_=0, rubber_end_value_=1 등)
}


    // 7) 최종 목표점 표시 및 정보 출력
    if (has_mid) {
        int target_px = int(center.x - final_target.y * scale_);
        int target_py = int(center.y - final_target.x * scale_);
        cv::circle(canvas, {target_px, target_py}, 8, cv::Scalar(0,255,0), -1); // 초록색
        
        float rubbercone_offset = -final_target.y * OFFSET_GAIN_;
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2)
           << "Target Offset: " << rubbercone_offset
           << " | L:" << left_group.size() << " R:" << right_group.size();
        cv::putText(
            canvas,
            ss.str(),
            cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(255,255,255),
            2
        );
    } else {
        cv::putText(
            canvas,
            "Insufficient cones detected",
            cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(255,255,255),
            2
        );
    }

    // 8) 화면 띄우기
    cv::imshow("Lidar View", canvas);
    cv::waitKey(1);
}

void LidarViewer::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::imshow("Camera View", img);
    cv::waitKey(1);
}

int main(int argc, char ** argv) {
    setenv("GDK_BACKEND", "x11", 1);
    setenv("QT_QPA_PLATFORM", "xcb", 1);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarViewer>());
    rclcpp::shutdown();
    return 0;
}
