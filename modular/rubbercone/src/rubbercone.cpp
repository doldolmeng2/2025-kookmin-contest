#include "rubbercone/rubbercone.hpp"
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <cstdlib>
#include <chrono>

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
    
    // 2) 50Hz 타이머 생성
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

    // 3) 유효 포인트만 수집 (각도와 거리 제한)
    const float ANG_MAX = 85.0f * M_PI/180.0f;
    std::vector<cv::Point2f> pts;
    float angle = msg->angle_min;
    for (float range : msg->ranges) {
        if (std::isfinite(range)
            && range >= 0.18f && range <= 0.80f
            && angle >= -ANG_MAX && angle <= ANG_MAX)
        {
            pts.emplace_back(range * std::cos(angle),
                             range * std::sin(angle));
        }
        angle += msg->angle_increment;
    }

    // 4) DBSCAN-like 클러스터링 (eps=0.08m)
    const float eps = 0.08f;
    std::vector<bool> visited(pts.size(), false);
    std::vector<std::vector<int>> clusters;
    for (size_t i = 0; i < pts.size(); ++i) {
        if (visited[i]) continue;
        std::vector<int> stack = {int(i)}, cluster;
        visited[i] = true;
        while (!stack.empty()) {
            int idx = stack.back(); stack.pop_back();
            cluster.push_back(idx);
            for (size_t j = 0; j < pts.size(); ++j) {
                if (visited[j]) continue;
                float dx = pts[j].x - pts[idx].x;
                float dy = pts[j].y - pts[idx].y;
                if (std::sqrt(dx*dx + dy*dy) <= eps) {
                    visited[j] = true;
                    stack.push_back(int(j));
                }
            }
        }
        clusters.push_back(std::move(cluster));
    }

    // 5) 클러스터별 가장 먼 점 → 빨간 점, 좌·우 근거리 대표점 추출
    float left_min = 1e6f, right_min = 1e6f;
    cv::Point2f left_pt, right_pt;
    for (auto &cluster : clusters) {
        float max_r = 0; int far_idx = -1;
        for (int idx : cluster) {
            float r = std::hypot(pts[idx].x, pts[idx].y);
            if (r > max_r) {
                max_r = r;
                far_idx = idx;
            }
        }
        if (far_idx < 0) continue;
        int px = int(center.x - pts[far_idx].y * scale_);
        int py = int(center.y - pts[far_idx].x * scale_);
        cv::circle(canvas, {px, py}, 4, cv::Scalar(0,0,255), -1);

        if (pts[far_idx].y > 0 && max_r < left_min) {
            left_min = max_r;
            left_pt  = pts[far_idx];
        }
        if (pts[far_idx].y < 0 && max_r < right_min) {
            right_min = max_r;
            right_pt  = pts[far_idx];
        }
    }

    // 6) 교점 계산 및 초록색 점으로 그리기
    bool has_mid = false;
    cv::Point2f mid;
    if (left_min < 1e6f && right_min < 1e6f) {
        // 양쪽 대표점이 모두 검출된 경우
        mid = { (left_pt.x + right_pt.x)/2, (left_pt.y + right_pt.y)/2 };
        float offset_m = - mid.y;
        rubber_offset_value_ = static_cast<int32_t>(offset_m * OFFSET_GAIN_);
        rubber_end_value_ = 0;
        has_mid = true;
    } else {
        // 왼쪽 또는 오른쪽 대표점이 하나라도 없으면 중앙점 계산 불가능
        has_mid = false;
        rubber_offset_value_ = 0;
        rubber_end_value_ = 1;
    }

    // 7) 화면에 중앙점 편차 텍스트 그리기
    if (has_mid) {
        int mid_px = int(center.x - mid.y * scale_);
        int mid_py = int(center.y - mid.x * scale_);
        cv::circle(canvas, {mid_px, mid_py}, 6, cv::Scalar(0,255,0), -1);
        // mid.x, mid.y 는 미터 단위. 원하는 포맷으로 예: 소수점 둘째 자리
        // 좌우 오프셋만 표시 (mid.y)
        float rubbercone_offset = - mid.y * OFFSET_GAIN_; // cm 단위로 변환
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2)
            << "Rubbercone Offset: " << rubbercone_offset;
        cv::putText(
            canvas,
            ss.str(),
            cv::Point(10, 30),            // 왼쪽 위OFFSET_GAIN
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,                          // 글자 크기
            cv::Scalar(255,255,255),      // 색상: 흰색
            2                             // 두께
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
