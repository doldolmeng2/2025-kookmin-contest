#ifndef RUBBERCONE_HPP
#define RUBBERCONE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <opencv2/opencv.hpp>

class LidarViewer : public rclcpp::Node {
public:
    LidarViewer();
private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void publishInfo();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr info_pub_;
    rclcpp::TimerBase::SharedPtr info_timer_;

    const int window_size_;
    const float scale_;
    const float OFFSET_GAIN_; // cm 단위로 변환을 위한 상수

    int32_t rubber_offset_value_;   // ← 최신 offset 값
    int32_t rubber_end_value_;      // ← rubbercone_end_ 플래그(0/1)
};

#endif // RUBBERCONE_HPP
