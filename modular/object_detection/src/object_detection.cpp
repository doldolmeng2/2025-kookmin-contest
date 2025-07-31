#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

using namespace std;
using namespace cv;
using std::placeholders::_1;

// ObjectDetector 클래스는 이미지 데이터를 처리하고, YOLOv8n 모델을 사용하여 객체를 감지하고 결과를 퍼블리시하는 ROS2 노드입니다.
class ObjectDetector : public rclcpp::Node {
public:
    ObjectDetector() : Node("object_detector_node") {
        // 이미지 구독 ("/resized_image" 토픽)
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/resized_image", 10, std::bind(&ObjectDetector::imageCallback, this, _1));

        // 모드 정보 구독 ("/mode_info" 토픽)
        mode_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/mode_info", 10, std::bind(&ObjectDetector::modeCallback, this, _1));

        // 객체 정보 퍼블리시 ("/object_info" 토픽)
        object_info_pub_ = this->create_publisher<std_msgs::msg::Int16>("/object_info", 10);

        // 오프셋 퍼블리시 ("/object_offset" 토픽)
        offset_pub_ = this->create_publisher<std_msgs::msg::Float32>("/object_offset", 10);

        // YOLOv8n ONNX 모델 로드
        net_ = dnn::readNet("/home/your_path/yolov8n.onnx");
        net_.setPreferableBackend(dnn::DNN_BACKEND_OPENCV);  // OpenCV 백엔드 사용
        net_.setPreferableTarget(dnn::DNN_TARGET_CPU);       // CPU 타겟 설정

        conf_threshold_ = 0.5;  // 신뢰도 임계값
        nms_threshold_ = 0.4;   // NMS(Non-Maximum Suppression) 임계값
        lane_type_ = 0;         // 차선 타입 (0: 1차선, 1: 2차선)
    }

private:
    // ROS2 구독자 및 퍼블리셔
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mode_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr object_info_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr offset_pub_;

    // YOLOv8 모델 및 파라미터
    dnn::Net net_;
    float conf_threshold_;
    float nms_threshold_;
    int lane_type_; // 0: 1차선, 1: 2차선

    // 모드 정보 콜백 함수 (차선 정보)
    void modeCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            lane_type_ = msg->data[1];  // 모드 정보에서 차선 타입 추출
        }
    }

    // 이미지 콜백 함수 (객체 감지 및 처리)
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // ROS 이미지를 OpenCV 이미지로 변환
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 예외: %s", e.what());
            return;
        }

        // 프레임 크기 추출
        Mat frame = cv_ptr->image;
        int frame_width = frame.cols;
        int frame_height = frame.rows;

        // YOLO 입력 처리
        Mat blob;
        dnn::blobFromImage(frame, blob, 1/255.0, Size(640, 640), Scalar(), true, false);  // YOLO 입력 크기 640x640으로 리사이즈
        net_.setInput(blob);  // 네트워크 입력으로 설정

        // 네트워크 출력 받기
        std::vector<Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());

        // 출력 후처리
        std::vector<Rect> boxes;        // 감지된 객체들의 바운딩 박스
        std::vector<float> confidences; // 각 객체의 신뢰도
        std::vector<int> class_ids;     // 객체의 클래스 ID

        for (size_t i = 0; i < outputs.size(); ++i) {
            float* data = (float*)outputs[i].data;
            for (int j = 0; j < outputs[i].rows; ++j, data += outputs[i].cols) {
                float conf = data[4];  // 객체 신뢰도
                if (conf >= conf_threshold_) {  // 신뢰도가 임계값 이상인 객체만 처리
                    float* scores = data + 5;  // 클래스 점수
                    Mat scores_mat(1, outputs[i].cols - 5, CV_32F, scores);
                    Point class_id_point;
                    double max_class_score;
                    minMaxLoc(scores_mat, 0, &max_class_score, 0, &class_id_point);

                    // 차량 클래스 (ID=2)만 필터링
                    if (max_class_score > conf_threshold_ && class_id_point.x == 2) { 
                        int cx = (int)(data[0] * frame_width); // 객체의 중심 X 좌표
                        int cy = (int)(data[1] * frame_height); // 객체의 중심 Y 좌표
                        int w = (int)(data[2] * frame_width);  // 객체의 너비
                        int h = (int)(data[3] * frame_height); // 객체의 높이

                        int left = cx - w / 2;  // 왼쪽 경계 계산
                        int top = cy - h / 2;   // 상단 경계 계산
                        boxes.emplace_back(left, top, w, h);  // 감지된 바운딩 박스 추가
                        confidences.push_back((float)conf);   // 신뢰도 추가
                        class_ids.push_back(class_id_point.x); // 클래스 ID 추가
                    }
                }
            }
        }

        // Non-Maximum Suppression(NMS) 적용
        std::vector<int> indices;
        dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);

        // 가장 가까운 차량 중심 기준으로 offset 계산
        float offset = 0.0f;
        int status_code = -1;  // 상태 코드 (-1: 계속 주행, -2: 회피 필요)

        if (!indices.empty()) {
            Rect nearest_box = boxes[indices[0]]; // 가장 가까운 차량 박스
            Point center = (nearest_box.br() + nearest_box.tl()) * 0.5; // 박스의 중심 계산
            int center_x = center.x;

            // 차선 정보에 따라 참조할 중심 X 좌표 설정
            int reference_center_x = frame_width / 2;
            if (lane_type_ == 0) reference_center_x = frame_width * 3 / 8; // 1차선
            else reference_center_x = frame_width * 5 / 8; // 2차선

            int delta_x = center_x - reference_center_x;  // 차량과 차선의 X 차이
            if (abs(delta_x) < frame_width / 10) {
                status_code = -2; // 회피 필요
                offset = (delta_x > 0 ? -1.0f : 1.0f) * 50.0f; // 방향에 따라 오프셋 설정
            } else {
                status_code = 100; // 거리값 예시, 추후 거리 추정 로직으로 대체
            }
        }

        // 상태 코드 퍼블리시
        std_msgs::msg::Int16 info_msg;
        info_msg.data = status_code;
        object_info_pub_->publish(info_msg);

        // 오프셋 퍼블리시
        std_msgs::msg::Float32 offset_msg;
        offset_msg.data = offset;
        offset_pub_->publish(offset_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // ROS2 초기화
    rclcpp::spin(std::make_shared<ObjectDetector>()); // ObjectDetector 노드 실행
    rclcpp::shutdown();  // ROS2 종료
    return 0;
}
