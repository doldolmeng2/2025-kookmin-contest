#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <xycar_msgs/msg/xycar_msg.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "main/control.hpp"
#include "main/resize.hpp"

using std::placeholders::_1;

class MainNode : public rclcpp::Node {
public:
    MainNode()
    : Node("main_node"),
      mode_(TRAFFIC_WAIT), lane_(0), rubbercone_offset_(0), end_flag_(0),
      lane_offset_(0), object_dist_(-1), traffic_green_(false)
    {
        // initial_mode 파라미터 선언 (기본: TRAFFIC_WAIT)
        this->declare_parameter<int>("initial_mode", TRAFFIC_WAIT);
        // 파라미터 읽어서 mode_에 저장
        mode_ = this->get_parameter("initial_mode").as_int();
        // 컨트롤러 파라미터 로드 (parameter.json 경로는 실행 환경에 맞게 수정)
        if (!controller_.loadParameters("/path/to/parameter.json")) {
            RCLCPP_ERROR(get_logger(), "파라미터 로드 실패");
        }

        // 퍼블리셔 설정
        resized_pub_ = create_publisher<sensor_msgs::msg::Image>("resized_image", 10);
        motor_pub_   = create_publisher<xycar_msgs::msg::XycarMsg>("xycar_motor", 10);
        mode_pub_    = create_publisher<std_msgs::msg::Int32MultiArray>("mode_info", 10);

        // 구독자 설정
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10, std::bind(&MainNode::imageCallback, this, _1)
        );
        rubbercone_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "rubbercone_info", 10, std::bind(&MainNode::rubberconeCallback, this, _1)
        );
        lane_sub_ = create_subscription<std_msgs::msg::Int16>(
            "lane_offset", 10, std::bind(&MainNode::laneOffsetCallback, this, _1)
        );
        object_sub_ = create_subscription<std_msgs::msg::Int16>(
            "object_info", 10, std::bind(&MainNode::objectInfoCallback, this, _1)
        );
        traffic_sub_ = create_subscription<std_msgs::msg::Bool>(
            "traffic_detection", 10, std::bind(&MainNode::trafficCallback, this, _1)
        );
    }

private:
    // 이미지 콜백: 리사이즈 후 퍼블리시
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // ROS Image -> OpenCV Mat 변환
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        // 리사이즈 처리
        cv::Mat resized = resizer_.resizeFrame(frame);
        // OpenCV Mat -> ROS Image 메시지 변환
        auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", resized).toImageMsg();
        resized_pub_->publish(*out_msg);
    }

    // 라바콘 정보 콜백: offset과 종료 플래그 업데이트
    void rubberconeCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            rubbercone_offset_ = msg->data[0];
            end_flag_ = msg->data[1];
            controlCycle();
        }
    }

    // 차선 오프셋 콜백
    void laneOffsetCallback(const std_msgs::msg::Int16::SharedPtr msg) {
        lane_offset_ = msg->data;
        controlCycle();
    }

    // 장애물 정보 콜백
    void objectInfoCallback(const std_msgs::msg::Int16::SharedPtr msg) {
        object_dist_ = msg->data;
        controlCycle();
    }

    // 신호등 감지 콜백
    void trafficCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        traffic_green_ = msg->data;
        controlCycle();
    }

    // 제어 사이클: 모드 결정, 컨트롤 호출, 퍼블리시
    void controlCycle() {
        // 모드 전환 로직
        if (mode_ == TRAFFIC_WAIT && traffic_green_) {
            mode_ = RUBBERCONE_DRIVE;
        } else if (mode_ == RUBBERCONE_DRIVE && end_flag_ == 1) {
            mode_ = RUBBERCONE_END;
        } else if (mode_ == RUBBERCONE_END) {
            mode_ = LANE_DRIVE;
        } else if (mode_ == LANE_DRIVE) {
            if (object_dist_ > 0) {
                mode_ = OBSTACLE_APPROACH;
            }
        }
        // 차선 변경 판단 (-2 입력 시)
        auto now = this->now();
        if (object_dist_ == -2 && mode_ == LANE_DRIVE) {
            mode_ = CHANGE_LANE;
            lane_change_time_ = now;
            lane_ = 1 - lane_;  // 1차선/2차선 토글
        } else if (mode_ == CHANGE_LANE) {
            // 3초 후 차선 변경 완료
            if ((now - lane_change_time_).seconds() > 3.0) {
                mode_ = LANE_DRIVE;
            }
        }

        // 컨트롤 입력값 결정
        int offset = 0;
        if (mode_ == RUBBERCONE_DRIVE) {
            offset = rubbercone_offset_;
        } else {
            offset = lane_offset_;
        }

        // 컨트롤러 업데이트
        controller_.update(mode_, offset, object_dist_);
        double angle = controller_.getAngle();
        double speed = controller_.getSpeed();

        // 모터 제어 메시지 퍼블리시
        xycar_msgs::msg::XycarMsg motor_msg;
        motor_msg.angle = static_cast<int16_t>(angle);
        motor_msg.speed = static_cast<int16_t>(speed);
        motor_pub_->publish(motor_msg);

        // 모드 정보 메시지 퍼블리시
        std_msgs::msg::Int32MultiArray mode_msg;
        mode_msg.data = {mode_, lane_};
        mode_pub_->publish(mode_msg);
    }

    // 멤버 변수
    Controller controller_;          // 제어 객체
    Resizer resizer_;             // 리사이즈 객체
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr resized_pub_;
    rclcpp::Publisher<xycar_msgs::msg::XycarMsg>::SharedPtr motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr mode_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr rubbercone_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr lane_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr object_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr traffic_sub_;

    int mode_;                    // 현재 모드
    int lane_;                    // 차선 정보 (0: 1차선, 1: 2차선)
    int rubbercone_offset_;       // 라바콘 오프셋
    int end_flag_;                // 라바콘 종료 플래그
    int lane_offset_;             // 차선 오프셋
    int object_dist_;             // 장애물 거리 또는 특수 신호(-2, -1)
    bool traffic_green_;          // 신호등 초록 여부
    rclcpp::Time lane_change_time_; // 차선 변경 시작 시간
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MainNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
