#include "main/main.hpp"

ImageSubscriberNode::ImageSubscriberNode()
: Node("image_subscriber_node")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10,
        std::bind(&ImageSubscriberNode::imageCallback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Subscribed to /image_raw");
}

void ImageSubscriberNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received image: width=%d, height=%d",
                msg->width, msg->height);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}