#include "rubbercone/rubbercone.hpp"

LidarViewer::LidarViewer()
: Node("rubbercone"), window_size_(800), scale_(1000.0f) { // scale_: meter to pixel
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&LidarViewer::scanCallback, this, std::placeholders::_1));
    
    cv::namedWindow("Lidar Viewer", cv::WINDOW_AUTOSIZE);
}

void LidarViewer::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    cv::Mat canvas = cv::Mat::zeros(window_size_, window_size_, CV_8UC3);
    cv::Point2f center(window_size_/2, window_size_/2);

    // Draw concentric circles every 10cm
    for (int r = 10; r <= (window_size_ / 2) / (scale_/100.0f); r += 10) {
        cv::circle(canvas, center, r * scale_/100.0f, cv::Scalar(100,100,100), 1);
        cv::putText(canvas, std::to_string(r)+"cm", 
                    cv::Point(center.x+5, center.y - r * scale_/100.0f), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 1);
    }

    // Plot LiDAR points
    float angle = msg->angle_min;
    for (const auto& range : msg->ranges) {
        if (std::isfinite(range) && range <= msg->range_max) {
            float x = range * std::cos(angle);
            float y = range * std::sin(angle);

            // Convert to pixel coordinates
            int px = static_cast<int>(center.x - y * scale_);
            int py = static_cast<int>(center.y - x * scale_);

            cv::circle(canvas, cv::Point(px, py), 2, cv::Scalar(0, 255, 0), -1);
        }
        angle += msg->angle_increment;
    }

    cv::imshow("Lidar Viewer", canvas);
    cv::waitKey(1);
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarViewer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
