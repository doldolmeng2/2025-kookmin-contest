#!/usr/bin/env python3

import rclpy, time
from rclpy.node import Node
from xycar_msgs.msg import XycarMotor

class DriverNode(Node):
    def __init__(self):
        super().__init__('driver')
        self.motor_publisher = self.create_publisher(XycarMotor, 'xycar_motor', 1)
        self.motor_msg = XycarMotor()

        # 파라미터 초기화
        self.speed = self.declare_parameter("speed", 50).value
        self.angle = 0
        
        self.get_logger().info('----- Xycar self-driving node started -----')

    def drive(self, angle, speed):
        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)
        self.motor_publisher.publish(self.motor_msg)

    def main_loop(self):
        while rclpy.ok():
            self.angle = 0
            self.drive(self.angle, self.speed)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    driver_node = DriverNode()

    try:
        driver_node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        driver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
