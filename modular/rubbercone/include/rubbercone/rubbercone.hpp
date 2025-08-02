#ifndef LIDAR_VIEWER_HPP
#define LIDAR_VIEWER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>

class LidarViewer : public rclcpp::Node {
public:
    LidarViewer();
private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    const int window_size_;
    const float scale_;
};

#endif // LIDAR_VIEWER_HPP
