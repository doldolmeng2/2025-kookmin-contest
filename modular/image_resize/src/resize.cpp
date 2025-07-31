#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ResizeNode : public rclcpp::Node {
public:
    ResizeNode() : Node("resize_node") {
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("resized_image", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            std::bind(&ResizeNode::callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Resize Node Started");
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // ROS Image -> OpenCV
            cv::Mat input_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // Resize to 640x360
            cv::Mat resized;
            cv::resize(input_image, resized, cv::Size(640, 360));

            // OpenCV -> ROS Image
            auto output_msg = cv_bridge::CvImage(msg->header, "bgr8", resized).toImageMsg();

            pub_->publish(*output_msg);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Resize error: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ResizeNode>());
    rclcpp::shutdown();
    return 0;
}
