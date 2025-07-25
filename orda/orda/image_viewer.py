#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # 토픽 이름은 상황에 따라 다를 수 있음
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
        self.get_logger().info("📷 이미지 시각화 노드 실행 중")

    def image_callback(self, msg):
        try:
            # ROS 이미지 → OpenCV 이미지로 변환 (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Camera", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"이미지 처리 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
