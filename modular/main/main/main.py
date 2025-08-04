import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int16, Bool
# from xycar_msgs.msg import XycarMotor
from main.control import Controller
import cv2
import numpy as np

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
        # self.motor_pub = self.create_publisher(XycarMotor, 'xycar_motor', 10)
        self.mode_pub = self.create_publisher(Int32MultiArray, 'mode_info', 10)

        # Subscribers (state updates only)
        self.create_subscription(Int32MultiArray, 'rubbercone_info', self.rubbercone_callback, 10)
        self.create_subscription(Int16, 'lane_offset',       self.lane_offset_callback, 10)
        self.create_subscription(Int16, 'object_info',        self.object_info_callback, 10)
        self.create_subscription(Bool,    'traffic_detection',self.traffic_callback, 10)
        self.create_subscription(Int16, 'object_distance', self.object_distance_callback, 10)

        # Variables
        self.lane = 0
        self.rubbercone_offset = 0
        self.end_flag = 0
        self.lane_offset = 0
        self.object_info = -1 # -1: not detected, 0: left, 1: right
        self.object_dist = 0 
        self.traffic_green = False
        self.rubbercone_end_time = None
        self.into_lane_timer = 2.0

        # 20 ms timer to run control cycle at ~50 Hz
        self.create_timer(0.02, self.control_cycle)

    def rubbercone_callback(self, msg):
        if len(msg.data) >= 2:
            self.rubbercone_offset = msg.data[0]
            self.end_flag          = msg.data[1]

    def lane_offset_callback(self, msg):
        self.lane_offset = msg.data

    def object_info_callback(self, msg):
        self.object_info = msg.data # object lane information -1:not detected  0:left 1 :right

    def traffic_callback(self, msg):
        self.traffic_green = msg.data # True if traffic light is green

    def object_distance_callback(self, msg):
        self.object_dist = msg.data # distance to the nearest object

    def control_cycle(self):
        now = self.get_clock().now()
        if self.rubbercone_end_time is not None:
            elapsed = (now - self.rubbercone_end_time).nanoseconds / 1e9
        # 모드 전환
        if self.mode == TRAFFIC_WAIT and self.traffic_green:
            self.mode = RUBBERCONE_DRIVE

        elif self.mode == RUBBERCONE_DRIVE and self.end_flag == 1:
            self.mode = RUBBERCONE_END
            self.rubbercone_end_time = now

        elif self.mode == RUBBERCONE_END and elapsed > self.into_lane_timer:
            self.mode = LANE_DRIVE

        elif self.mode == LANE_DRIVE and self.object_dist != 0: # object_dist need reset
            self.mode = OBSTACLE_APPROACH

        elif self.mode == OBSTACLE_APPROACH and self.object_info != -1: # object_info is detected
            if self.lane == self.object_info :  # same side 
                self.lane = 1 - self.object_info # change lane
                self.mode = CHANGE_LANE
            else:                               # different side
                self.mode = LANE_DRIVE
            self.object_dist = 0 # reset object distance
            self.object_info = -1 # reset object info

        elif self.mode == CHANGE_LANE and self.is_change_end():
            self.mode = LANE_DRIVE
            
        # 오프셋 선택
        offset = self.rubbercone_offset if self.mode == RUBBERCONE_DRIVE else self.lane_offset

        # Controller 업데이트
        self.controller.update(self.mode, offset, self.object_dist)
        angle = self.controller.get_angle()
        speed = self.controller.get_speed()

        # 모터 제어 메시지 퍼블리시
        # motor_msg = XycarMotor()
        # motor_msg.angle = int(angle)
        # motor_msg.speed = int(speed)
        # self.motor_pub.publish(motor_msg)

        # 모드 정보 퍼블리시
        mode_msg = Int32MultiArray()
        mode_msg.data = [self.mode, self.lane]
        self.mode_pub.publish(mode_msg)

        # log: 화면에 상태 텍스트 그리기
        # 1) 빈 화면 초기화
        log_img = np.zeros((300, 600, 3), dtype=np.uint8)

        # 2) 변수 문자열 변환
        mode_map = {
            TRAFFIC_WAIT:      'TRAFFIC_WAIT',
            RUBBERCONE_DRIVE:  'RUBBERCONE_DRIVE',
            RUBBERCONE_END:    'RUBBERCONE_END',
            LANE_DRIVE:        'LANE_DRIVE',
            OBSTACLE_APPROACH: 'OBSTACLE_APPROACH',
            CHANGE_LANE:       'CHANGE_LANE',
        }
        mode_str        = f"Mode: {mode_map.get(self.mode, 'UNKNOWN')}"
        lane_str        = f"{'Lane 1' if self.lane==0 else 'Lane 2'}"
        endflag_str     = 'Rubber End' if self.end_flag==1 else 'Rubber Not End'
        objinfo_map     = {
            -1: 'Not Detected',
             0: 'Obstacle left',
             1: 'Obstacle right',
        }
        objinfo_str     = objinfo_map.get(self.object_info, 'UNKNOWN')
        offset_str      = f"Offset: {offset}"
        objdist_str     = f"Object dist: {self.object_dist}"
        angle_str       = f"Angle: {angle:.1f}"
        speed_str       = f"Speed: {speed:.1f}"

        # 3) 화면에 그리기
        y0, dy = 30, 30
        for i, text in enumerate([mode_str, angle_str, speed_str, offset_str, lane_str,
                                  endflag_str, objinfo_str,
                                  objdist_str]):
            cv2.putText(
                log_img, text,
                (10, y0 + i*dy),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (255,255,255), 2
            )

        # 4) 보여주기 및 리셋
        cv2.imshow('Status', log_img)
        cv2.waitKey(1)


    def is_change_end(self):
        return True if abs(self.lane_offset) < 30 else False


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
