#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ResizeNode : public rclcpp::Node {
public:
    ResizeNode() : Node("resize_node") {
        // 1) 공통 QoS: 이미지/센서용 Best Effort
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();              // ← 핵심 (Reliable → Best Effort)
        qos.durability_volatile();      // 일반적으로 이미지 토픽은 VOLATILE

        // 2) 퍼블리셔/구독자에 각각 적용
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("resized_image", qos);

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", qos,
            std::bind(&ResizeNode::callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Resize Node Started (QoS: BestEffort)");
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat input_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

            cv::Mat resized;
            cv::resize(input_image, resized, cv::Size(640, 360));

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
