#pragma once
// #pragma once : 헤더 가드 역할을 함.
// 동일한 헤더 파일이 여러 번 include되는 것을 방지.

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
// sensor_msgs 패키지의 Image 메시지 타입 포함.

class ImageSubscriberNode : public rclcpp::Node
// ImageSubscriberNode 클래스 정의.
// rclcpp::Node를 상속받아 ROS2 노드로 동작.
// → 즉, ROS2 네트워크에서 구독, 퍼블리시, 서비스 등 가능.
{
public:
    ImageSubscriberNode();
    // 생성자 선언.
    // 실제 구현은 main.cpp에서 정의하며, 노드 이름 설정 및 토픽 구독 초기화 담당.

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    // 콜백 함수 선언.
    // /image_raw 토픽에서 새로운 이미지 메시지가 수신될 때마다 호출됨.
    // msg는 sensor_msgs::msg::Image 타입의 SharedPtr.

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    // 구독(subscription) 객체를 저장할 변수.
    // SharedPtr로 관리하여 메모리 안전성 보장.
    // 생성자는 create_subscription() 호출 시 초기화함.
};
