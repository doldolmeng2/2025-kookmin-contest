#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

// HLS 임계값
int h_hls_low=0, l_hls_low=0, s_hls_low=0;
int h_hls_high= 179, l_hls_high=255, s_hls_high=255;

// HSV 임계값
int h_hsv_low = 0, s_hsv_low = 0, v_hsv_low = 0;
int h_hsv_high = 179, s_hsv_high = 255, v_hsv_high = 255;

void on_trackbar(int, void*) {}

class HSLHSVTunerNode : public rclcpp::Node {
public:
    HSLHSVTunerNode() : Node("hsl_hsv_tuner_node") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&HSLHSVTunerNode::imageCallback, this, std::placeholders::_1)
        );

        // HLS 트랙바
        namedWindow("HLS Mask", WINDOW_NORMAL);
        resizeWindow("HLS Mask", 640, 360);
        createTrackbar("H low",  "HLS Mask", &h_hls_low,   179, on_trackbar);
        createTrackbar("H high", "HLS Mask", &h_hls_high,  179, on_trackbar);
        createTrackbar("L low",  "HLS Mask", &l_hls_low,   255, on_trackbar);
        createTrackbar("L high", "HLS Mask", &l_hls_high,  255, on_trackbar);
        createTrackbar("S low",  "HLS Mask", &s_hls_low,   255, on_trackbar);
        createTrackbar("S high", "HLS Mask", &s_hls_high,  255, on_trackbar);

        // HSV 트랙바
        namedWindow("HSV Mask", WINDOW_NORMAL);
        resizeWindow("HSV Mask", 640, 360);
        createTrackbar("H low", "HSV Mask", &h_hsv_low, 179, on_trackbar);
        createTrackbar("H high", "HSV Mask", &h_hsv_high, 179, on_trackbar);
        createTrackbar("S low", "HSV Mask", &s_hsv_low, 255, on_trackbar);
        createTrackbar("S high", "HSV Mask", &s_hsv_high, 255, on_trackbar);
        createTrackbar("V low", "HSV Mask", &v_hsv_low, 255, on_trackbar);
        createTrackbar("V high", "HSV Mask", &v_hsv_high, 255, on_trackbar);
        
        namedWindow("Combined Mask", WINDOW_NORMAL);
        resizeWindow("Combined Mask", 640, 360);
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

        // 1) HLS 마스크
        Mat hls, mask_hls;
        cvtColor(frame, hls, COLOR_BGR2HLS);  // OpenCV는 HLS 사용
        inRange(hls,
                Scalar(h_hls_low, l_hls_low, s_hls_low),
                Scalar(h_hls_high, l_hls_high, s_hls_high),
                mask_hls);

        // 2) HSV 마스크
        Mat hsv, mask_hsv;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        inRange(hsv,
                Scalar(h_hsv_low, s_hsv_low, v_hsv_low),
                Scalar(h_hsv_high, s_hsv_high, v_hsv_high),
                mask_hsv);

        // 3) 두 마스크를 AND로 결합 (HLS → HSV 순차 필터링과 동일한 결과)
        Mat combined;
        bitwise_and(mask_hls, mask_hsv, combined);

        imshow("HLS Mask", mask_hls);
        imshow("HSV Mask", mask_hsv);
        imshow("Combined Mask", combined);

        waitKey(1);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HSLHSVTunerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}