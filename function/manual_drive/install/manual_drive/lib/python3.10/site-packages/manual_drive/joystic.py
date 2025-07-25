#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from xycar_msgs.msg import XycarMotor
import time

class JoyToMotor(Node):
    def __init__(self):
        super().__init__('joy_to_motor')

        # joy 메시지 구독
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # motor 퍼블리셔
        self.motor_pub = self.create_publisher(XycarMotor, 'xycar_motor', 10)
        self.motor_msg = XycarMotor()

        # 초기값
        self.axis_0 = 0.0  # 조향
        self.axis_4 = 0.0  # 속도

        # 타이머: 0.1초마다 motor 메시지 퍼블리시
        self.timer = self.create_timer(0.1, self.publish_motor)

        self.get_logger().info("🚗 Joy → Motor 노드 시작됨 (10Hz publishing)")

    def joy_callback(self, msg):
        if len(msg.axes) > 4:
            self.axis_0 = msg.axes[0]
            self.axis_4 = msg.axes[4]

    def publish_motor(self):
        # 계산
        angle = -50 * self.axis_0
        speed = 50 * self.axis_4

        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)

        # 퍼블리시
        self.motor_pub.publish(self.motor_msg)
        print(f"[PUBLISH] Angle: {angle:.2f}, Speed: {speed:.2f}", flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToMotor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
