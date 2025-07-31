import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int16, Bool
from xycar_msgs.msg import XycarMotor
from control import Controller

# 모드 상수 정의
TRAFFIC_WAIT = 0
RUBBERCONE_DRIVE = 1
RUBBERCONE_END = 2
LANE_DRIVE = 3
OBSTACLE_APPROACH = 4
CHANGE_LANE = 5

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')

        # Parameter
        self.declare_parameter('initial_mode', TRAFFIC_WAIT)
        self.mode = self.get_parameter('initial_mode').value

        # Controller
        self.controller = Controller(self)

        # Publishers
        self.motor_pub = self.create_publisher(XycarMotor, 'xycar_motor', 10)
        self.mode_pub = self.create_publisher(Int32MultiArray, 'mode_info', 10)

        # Subscribers (state updates only)
        self.create_subscription(Int32MultiArray, 'rubbercone_info', self.rubbercone_callback, 10)
        self.create_subscription(Int16, 'lane_offset',       self.lane_offset_callback, 10)
        self.create_subscription(Int16, 'object_info',        self.object_info_callback, 10)
        self.create_subscription(Bool,    'traffic_detection',self.traffic_callback, 10)

        # Variables
        self.lane = 0
        self.rubbercone_offset = 0
        self.end_flag = 0
        self.lane_offset = 0
        self.object_dist = -1
        self.traffic_green = False
        self.lane_change_time = None

        # 20 ms timer to run control cycle at ~50 Hz
        self.create_timer(0.02, self.control_cycle)

    def rubbercone_callback(self, msg):
        if len(msg.data) >= 2:
            self.rubbercone_offset = msg.data[0]
            self.end_flag          = msg.data[1]

    def lane_offset_callback(self, msg):
        self.lane_offset = msg.data

    def object_info_callback(self, msg):
        self.object_dist = msg.data

    def traffic_callback(self, msg):
        self.traffic_green = msg.data

    def control_cycle(self):
        # 모드 전환
        if self.mode == TRAFFIC_WAIT and self.traffic_green:
            self.mode = RUBBERCONE_DRIVE
        elif self.mode == RUBBERCONE_DRIVE and self.end_flag == 1:
            self.mode = RUBBERCONE_END
        elif self.mode == RUBBERCONE_END:
            self.mode = LANE_DRIVE
        elif self.mode == LANE_DRIVE and self.object_dist > 0:
            self.mode = OBSTACLE_APPROACH

        now = self.get_clock().now()
        if self.object_dist == -2 and self.mode == LANE_DRIVE:
            self.mode            = CHANGE_LANE
            self.lane_change_time = now
            self.lane            = 1 - self.lane
        elif self.mode == CHANGE_LANE and (now - self.lane_change_time).nanoseconds / 1e9 > 3.0:
            self.mode = LANE_DRIVE

        # 오프셋 선택
        offset = self.rubbercone_offset if self.mode == RUBBERCONE_DRIVE else self.lane_offset

        # Controller 업데이트
        self.controller.update(self.mode, offset, self.object_dist)
        angle = self.controller.get_angle()
        speed = self.controller.get_speed()

        # 모터 제어 메시지 퍼블리시
        motor_msg = XycarMotor()
        motor_msg.angle = int(angle)
        motor_msg.speed = int(speed)
        self.motor_pub.publish(motor_msg)

        # 모드 정보 퍼블리시
        mode_msg = Int32MultiArray()
        mode_msg.data = [self.mode, self.lane]
        self.mode_pub.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
