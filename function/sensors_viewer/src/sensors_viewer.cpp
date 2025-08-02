#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class SensorsViewerNode : public rclcpp::Node
{
public:
    SensorsViewerNode() : Node("sensors_viewer")
    {
        // 구독자 초기화
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/resized_image", 10,
            std::bind(&SensorsViewerNode::imageCallback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),  
            std::bind(&SensorsViewerNode::scanCallback, this, std::placeholders::_1));

            
        RCLCPP_INFO(this->get_logger(), "Sensor viewer node started. Subscribing to /image_raw and /scan");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            cv::imshow("Camera View", frame);
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int width = 700, height = 700; // ✅ 창 크기 확대
        float max_range = 2.0;         // ✅ 최대 표시 거리 2m
        float scale = (width / 2 - 50) / max_range; // ✅ 비율 조정 (가장자리 여유)

        cv::Mat lidar_img(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Point center(width / 2, height / 2);

        float angle = msg->angle_min;
        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            float r = msg->ranges[i];
            if (std::isfinite(r) && r <= max_range)
            {
                // Polar → Cartesian 변환
                int x = static_cast<int>(center.x + r * scale * cos(angle));
                int y = static_cast<int>(center.y - r * scale * sin(angle));
                cv::circle(lidar_img, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1);
            }
            angle += msg->angle_increment;
        }

        // 중앙점 강조
        cv::circle(lidar_img, center, 6, cv::Scalar(0, 0, 255), -1);

        // 원형 가이드 라인 (0.5m 간격)
        for (int i = 1; i <= 4; i++)
        {
            int radius = static_cast<int>(i * 0.5 * scale);
            cv::circle(lidar_img, center, radius, cv::Scalar(50, 50, 50), 1);
        }

        cv::imshow("LiDAR View (max 2m)", lidar_img);
        cv::waitKey(1);
}


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorsViewerNode>());
    rclcpp::shutdown();
    return 0;
}
