#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int h_low = 0, s_low = 0, v_low = 0;
int h_high = 180, s_high = 255, v_high = 255;

void on_trackbar(int, void*) {}

class HSVTunerNode : public rclcpp::Node {
public:
    HSVTunerNode() : Node("hsv_tuner_node") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&HSVTunerNode::imageCallback, this, std::placeholders::_1)
        );

        namedWindow("HSV Mask", WINDOW_NORMAL);
        resizeWindow("HSV Mask", 640, 360);
        createTrackbar("H low", "HSV Mask", &h_low, 180, on_trackbar);
        createTrackbar("H high", "HSV Mask", &h_high, 180, on_trackbar);
        createTrackbar("S low", "HSV Mask", &s_low, 255, on_trackbar);
        createTrackbar("S high", "HSV Mask", &s_high, 255, on_trackbar);
        createTrackbar("V low", "HSV Mask", &v_low, 255, on_trackbar);
        createTrackbar("V high", "HSV Mask", &v_high, 255, on_trackbar);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 예외: %s", e.what());
            return;
        }

        Mat frame = cv_ptr->image;
        resize(frame, frame, Size(640, 360));

        Mat hsv, mask;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        inRange(hsv,
                Scalar(h_low, s_low, v_low),
                Scalar(h_high, s_high, v_high),
                mask);

        imshow("Original", frame);
        imshow("HSV Mask", mask);

        // 콘솔 출력
        cout << "\rHSV 범위: Scalar(" << h_low << ", " << s_low << ", " << v_low
             << ") ~ Scalar(" << h_high << ", " << s_high << ", " << v_high << ")      " << flush;

        waitKey(1);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HSVTunerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

