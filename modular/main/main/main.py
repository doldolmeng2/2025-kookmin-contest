# main.py
# ROS2 메인 제어 노드. LiDAR(object_detection.cpp)에서 발행하는 /object_info(Float32MultiArray)
# 및 별도 거리 토픽 /object_distance(Float32)를 모두 구독하여 상태머신 모드 전환과 속도/조향을 결정한다.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int16, Bool
from std_msgs.msg import Float32, Float32MultiArray
from main.control import Controller
import cv2
import numpy as np
from sensor_msgs.msg import Joy

# 모드 상수
TRAFFIC_WAIT       = 0
RUBBERCONE_DRIVE   = 1
RUBBERCONE_END     = 2
LANE_DRIVE         = 3
OBSTACLE_APPROACH  = 4
CHANGE_LANE        = 5

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')

        # 파라미터
        self.declare_parameter('mode', TRAFFIC_WAIT)
        self.mode = self.get_parameter('mode').value

        # 컨트롤러
        self.controller = Controller(self)

        # 퍼블리셔
        self.motor_pub = self.create_publisher(Float32MultiArray, 'xycar_motor', 10)
        self.mode_pub  = self.create_publisher(Int32MultiArray, 'mode_info', 10)

        # 서브스크라이버
        # 1) 라바콘: [offset, end_flag]
        self.create_subscription(Int32MultiArray, 'rubbercone_info',    self.rubbercone_callback, 10)
        # 2) 차선 오프셋(Int16)
        self.create_subscription(Int16,            'lane_offset',        self.lane_offset_callback, 10)
        # 3) 객체 정보(Float32MultiArray): [exists, min_dist, angle, span, cluster_size]
        self.create_subscription(Float32MultiArray,'object_info',        self.object_info_callback, 10)
        # 4) 신호등(Bool)
        self.create_subscription(Bool,             'traffic_detection',  self.traffic_callback, 10)
        # 5) 별도 거리(Float32) — 사용자가 발행하는 실수 거리. 모드 전환 트리거로도 사용.
        self.create_subscription(Float32,          'object_distance',    self.object_distance_callback, 10)
        # 6) Xbox 컨트롤러
        self.create_subscription(Joy,              'joy',                self.joy_callback, 10)

        # 조이스틱 디바운스
        self.prev_x = 0
        self.prev_b = 0

        # 테스트 모드(자동 모드 전환 비활성화 플래그). 필요 시 False로.
        self.test_mode = True

        # 상태 변수
        self.lane                = 0
        self.rubbercone_offset   = 0
        self.end_flag            = 0
        self.lane_offset         = 0

        # object_info 최신값
        self.obj_exists   = 0.0
        self.obj_min_dist = float('inf')
        self.obj_angle    = 0.0
        self.obj_span     = 0.0
        self.obj_cluster  = 0.0

        # 별도 거리(Float32)
        self.object_dist         = 0.0

        self.traffic_green       = False
        self.rubbercone_end_time = None
        self.into_lane_timer     = 2.0
        self.lane_drive_started  = False
        self.lane_drive_start_time = None

        # 50Hz 제어 루프
        self.create_timer(0.02, self.control_cycle)

    # -----------------------------
    # 콜백
    # -----------------------------
    def joy_callback(self, msg: Joy):
        # X(2) rising → mode--
        # B(1) rising → mode++
        x = msg.buttons[2]
        b = msg.buttons[1]
        if x == 1 and self.prev_x == 0:
            self.end_flag = 0
            self.rubbercone_end_time = None
            self.mode = max(TRAFFIC_WAIT, self.mode - 1)
        if b == 1 and self.prev_b == 0:
            self.mode = min(CHANGE_LANE, self.mode + 1)
        self.prev_x = x
        self.prev_b = b

    def rubbercone_callback(self, msg: Int32MultiArray):
        # msg.data = [offset, end_flag]
        if len(msg.data) >= 2:
            self.rubbercone_offset = int(msg.data[0])
            self.end_flag          = int(msg.data[1])

    def lane_offset_callback(self, msg: Int16):
        self.lane_offset = int(msg.data)

    def object_info_callback(self, msg: Float32MultiArray):
        # data = [exists, min_dist, angle, span, cluster_size]
        data = msg.data
        if not data or len(data) < 5:
            return
        self.obj_exists   = float(data[0])
        self.obj_min_dist = float(data[1])
        self.obj_angle    = float(data[2])
        self.obj_span     = float(data[3])
        self.obj_cluster  = float(data[4])

        # object_info 수신시 즉시 모드 전환 검사
        self.mode = self.check_and_switch_mode(self.mode)

    def traffic_callback(self, msg: Bool):
        self.traffic_green = bool(msg.data)

    def object_distance_callback(self, msg: Float32):
        # 사용자가 발행하는 실수형 거리(m). 0보다 크면 존재 플래그 1.0로 보고 즉시 모드 전환 검사.
        self.object_dist   = float(msg.data)
        self.obj_exists    = 1.0 if self.object_dist > 0.0 else 0.0
        self.obj_min_dist  = float(self.object_dist)
        # cluster_size가 없는 단독 거리 입력인 경우 최소 1로 가정
        self.obj_cluster   = max(self.obj_cluster, 1.0) if self.obj_exists >= 0.5 else 0.0

        # 거리만으로도 접근 조건이면 즉시 전환
        self.mode = self.check_and_switch_mode(self.mode)

    # -----------------------------
    # 모드 전환 규칙
    # -----------------------------
    def check_and_switch_mode(self, mode: int) -> int:
        # 조건: exists == 1, min_dist < 1.0m, cluster_size < 15
        cond_exists  = self.obj_exists >= 0.5
        cond_dist    = self.obj_min_dist < 1.0
        cond_cluster = self.obj_cluster < 15.0
        if cond_exists and cond_dist and cond_cluster:
            return OBSTACLE_APPROACH
        return mode

    # -----------------------------
    # 제어 루프
    # -----------------------------
    def control_cycle(self):
        now = self.get_clock().now()
        elapsed = (now - self.rubbercone_end_time).nanoseconds / 1e9 if self.rubbercone_end_time else 0.0

        # 임시(수동 시나리오)
        if self.mode == TRAFFIC_WAIT and self.traffic_green:
            self.mode = RUBBERCONE_DRIVE
        elif self.mode == RUBBERCONE_DRIVE and self.end_flag == 1:
            self.mode = RUBBERCONE_END
            self.rubbercone_end_time = now

        # 자동 FSM (test_mode=False일 때만)
        if not self.test_mode:
            if self.mode == TRAFFIC_WAIT and self.traffic_green:
                self.mode = RUBBERCONE_DRIVE
            elif self.mode == RUBBERCONE_DRIVE and self.end_flag == 1:
                self.mode = RUBBERCONE_END
                self.rubbercone_end_time = now
            elif self.mode == RUBBERCONE_END and elapsed > self.into_lane_timer:
                self.mode = LANE_DRIVE

            # object_info/ object_distance 반영한 강제 전환 검사
            self.mode = self.check_and_switch_mode(self.mode)

            # 차선 변경 완료 판정
            if self.mode == CHANGE_LANE and self.is_change_end():
                self.mode = LANE_DRIVE

        # 오프셋 선택
        offset = self.rubbercone_offset if self.mode == RUBBERCONE_DRIVE else self.lane_offset

        # 컨트롤러 업데이트
        # 장애물 접근 속도 PI 제어는 control.Controller._compute_obstacle_speed에서 object_dist 사용
        self.controller.update(self.mode, offset, int(self.object_dist))
        angle = self.controller.get_angle()
        speed = self.controller.get_speed()

        # .

        # LANE_DRIVE 진입 후 속도 제한(가속 램프용)
        if not self.lane_drive_started:
            self.lane_drive_started   = True
            self.lane_drive_start_time = now
        if self.mode == TRAFFIC_WAIT:
            self.lane_drive_started   = False
            self.lane_drive_start_time = None
        if self.mode == LANE_DRIVE and self.lane_drive_start_time is not None:
            elapsed_lane = (now - self.lane_drive_start_time).nanoseconds / 1e9
            if elapsed_lane < 8.0:
                speed = 10.0

        # 퍼블리시
        motor_msg = Float32MultiArray()
        motor_msg.data = [float(angle), float(speed)]
        self.motor_pub.publish(motor_msg)

        mode_msg = Int32MultiArray()
        mode_msg.data = [self.mode, self.lane]
        self.mode_pub.publish(mode_msg)

        # 상태 시각화
        self.draw_status(angle, speed, offset)

    # -----------------------------
    # 유틸
    # -----------------------------
    def is_change_end(self):
        # TODO: 실제 차선 변경 완료 조건 구현 필요
        return True if abs(self.lane_offset) < 0 else False

    def draw_status(self, angle: float, speed: float, offset: int):
        img = np.zeros((320, 860, 3), dtype=np.uint8)
        mode_map = {
            TRAFFIC_WAIT:      'TRAFFIC_WAIT',
            RUBBERCONE_DRIVE:  'RUBBERCONE_DRIVE',
            RUBBERCONE_END:    'RUBBERCONE_END',
            LANE_DRIVE:        'LANE_DRIVE',
            OBSTACLE_APPROACH: 'OBSTACLE_APPROACH',
            CHANGE_LANE:       'CHANGE_LANE',
        }
        lines = [
            f"Mode: {mode_map.get(self.mode,'UNKNOWN')}",
            f"Angle: {angle:.1f}",
            f"Speed: {speed:.1f}",
            f"Offset: {offset}",
            f"Lane: {'Lane 1' if self.lane==0 else 'Lane 2'}",
            f"Rubber: {'End' if self.end_flag==1 else 'Not End'}",
            f"Obj exists: {int(self.obj_exists)}  min_dist[m]: {self.obj_min_dist:.2f}",
            f"Obj csize: {self.obj_cluster:.0f}  angle[rad]: {self.obj_angle:.2f}  span[rad]: {self.obj_span:.2f}",
            f"Object dist(Float32): {self.object_dist:.2f}",
        ]
        y0, dy = 30, 30
        for i, t in enumerate(lines):
            cv2.putText(img, t, (10, y0 + i*dy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.imshow('Status', img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
