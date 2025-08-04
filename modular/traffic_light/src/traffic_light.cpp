#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class TrafficLightDetector : public rclcpp::Node
{
public:
  TrafficLightDetector()
  : Node("traffic_light_detector")
  {
    // 퍼블리셔
    pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/traffic_detection", 10);

    // 서브스크라이버
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/resized_image", 10,
      std::bind(&TrafficLightDetector::image_callback, this, std::placeholders::_1));

    ////////////  임계치 설정  ///////////////
    // 녹색 픽셀 검출 임계치 0.01 = 1%
    threshold_ratio_ = 0.1;
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // cv::Mat으로 변환
    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge 예외: %s", e.what());
      return;
    }

    // ROI: 상단 1/3
    int h = bgr.rows;
    int w = bgr.cols;
    cv::Rect roi(0, 0, w, h / 3);
    cv::Mat top_third = bgr(roi);

    // BGR → HSV 변환
    cv::Mat hsv;
    cv::cvtColor(top_third, hsv, cv::COLOR_BGR2HSV);

    // 녹색 범위 (Hue: 35~85, Sat:100~255, Val:100~255

    ////////////  HSV값 설정  ///////////////
    cv::Scalar lower_green(35, 100, 100);
    cv::Scalar upper_green(85, 255, 255);
    cv::Mat mask;
    cv::inRange(hsv, lower_green, upper_green, mask);

    // 녹색 픽셀 수
    int green_pixels = cv::countNonZero(mask);
    int total_pixels = mask.rows * mask.cols;
    bool detected = (green_pixels > total_pixels * threshold_ratio_);

    // 결과 퍼블리시
    std_msgs::msg::Bool out;
    out.data = detected;
    pub_->publish(out);

    RCLCPP_DEBUG(this->get_logger(),
                 "Green pixels: %d (%.2f%%) → %s",
                 green_pixels,
                 100.0 * green_pixels / total_pixels,
                 detected ? "TRUE" : "FALSE");
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  double threshold_ratio_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrafficLightDetector>());
  rclcpp::shutdown();
  return 0;
}