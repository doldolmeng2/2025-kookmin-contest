import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int16, Bool
from std_msgs.msg import Float32MultiArray
from main.control import Controller
import cv2
import time
import numpy as np
from sensor_msgs.msg import Joy

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
        self.declare_parameter('mode', TRAFFIC_WAIT)
        self.mode = self.get_parameter('mode').value
        self.last_change_time = 0.0 # 타이머
        self.last_log_time = 0.0 # 로그타이머
        self.cond_count = 0
        self.cond_threshold = 5  # 몇 프레임 이상 유지할지 (예: 5프레임)

        # Controller
        self.controller = Controller(self)

        # Publishers
        self.motor_pub = self.create_publisher(Float32MultiArray, 'xycar_motor', 10)
        self.mode_pub = self.create_publisher(Int32MultiArray, 'mode_info', 10)

        # Subscribers (state updates only)
        self.create_subscription(Int32MultiArray, 'rubbercone_info', self.rubbercone_callback, 10)
        self.create_subscription(Int16, 'lane_offset',       self.lane_offset_callback, 10)
        self.create_subscription(Float32MultiArray, 'object_info',        self.object_info_callback, 10)
        self.create_subscription(Bool,    'traffic_detection',self.traffic_callback, 10)
        # self.create_subscription(Int32MultiArray, 'object_distance', self.object_distance_callback, 10)

        # Xbox 컨트롤러 조이스틱 토픽 구독
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        # 버튼 디바운스용 이전 상태
        self.prev_x = 0
        self.prev_b = 0

        # test mode 파라미터
        self.test_mode = False # (오상영 디버깅) True -> False
        # Variables
        self.lane = 0
        self.rubbercone_offset = 0
        self.end_flag = 0
        self.lane_offset = 0
        self.object_info = -1 # -1: not detected, 0: left, 1: right
        self.object_dist = 0
        self.traffic_green = False
        self.rubbercone_end_time = None
        self.into_lane_timer = 1.7 # 라바콘 끝나고 하드코딩 시간초
        self.lane_drive_started = False
        

        # 20 ms timer to run control cycle at ~50 Hz
        self.create_timer(0.02, self.control_cycle)

        self.lane_drive_start_time = None

        # Variables (기존 아래 줄들 바로 근처에 추가)
        self.obj_exists  = 0.0
        self.obj_angle   = 0.0
        self.obj_span    = 0.0
        self.obj_cluster = 1e9   # 아주 크게: '안전' 쪽으로 평가되게
        self.object_dist = 1e9   # 아주 멀다로 초기화

    def joy_callback(self, msg: Joy):
        """
        X 버튼(x축 인덱스 2) 누르면 mode--,
        B 버튼(인덱스 1) 누르면 mode++.
        rising edge 만 처리하고 0~5 로 클램프.
        """
        x = msg.buttons[2]
        b = msg.buttons[1]
        # X 버튼 rising edge → mode--
        if x == 1 and self.prev_x == 0:
            self.end_flag = 0 # X 버튼 누르면 라바콘 종료 플래그 초기화
            self.rubbercone_end_time = None # 라바콘 종료 시간 초기화
            self.mode = max(TRAFFIC_WAIT, self.mode - 1)
            self.get_logger().info(f"Mode-- -> {self.mode}")
        # B 버튼 rising edge → mode++
        if b == 1 and self.prev_b == 0:
            self.mode = min(CHANGE_LANE, self.mode + 1)
            self.get_logger().info(f"Mode++ -> {self.mode}")
        self.prev_x = x
        self.prev_b = b

    def rubbercone_callback(self, msg):
        if len(msg.data) >= 2:
            self.rubbercone_offset = msg.data[0]
            self.end_flag          = msg.data[1]

    def lane_offset_callback(self, msg):
        self.lane_offset = msg.data

#    def object_info_callback(self, msg):
#        self.object_info = msg.data # object lane information -1:not detected  0:left 1 :right

    def object_info_callback(self, msg: Float32MultiArray):
        # data = [exists, min_dist, angle, span, cluster_size]
        data = msg.data
        self.obj_exists   = float(data[0])
        self.object_dist = float(data[1])
        self.obj_angle    = float(data[2])
        self.obj_span     = float(data[3])
        self.obj_cluster  = float(data[4])
        self.box_size     = flost(data[5])

    def traffic_callback(self, msg):
        self.traffic_green = msg.data # True if traffic light is green

    # def object_distance_callback(self, msg):
    #     self.object_dists = msg.data # distance to the nearest object

    def control_cycle(self):
        now = self.get_clock().now()
        if self.rubbercone_end_time is not None:
            elapsed = (now - self.rubbercone_end_time).nanoseconds / 1e9
        else:
            elapsed = 0.0

        #################### 임시 코드 (라바콘) ###################
        if self.mode == TRAFFIC_WAIT and self.traffic_green:
            self.mode = RUBBERCONE_DRIVE
            self.get_logger().info("초록불 감지 -> 라바콘 모드로 변경")

        elif self.mode == RUBBERCONE_DRIVE and self.end_flag == 1:
            self.mode = RUBBERCONE_END
            self.rubbercone_end_time = now
            self.get_logger().info("라바콘 종료 차선 진입")

        elif self.mode == RUBBERCONE_END and elapsed > self.into_lane_timer:
            self.mode = TRAFFIC_WAIT
        ###################################################


        # 모드 전환
        if not self.test_mode: # test mode가 아닐 때만 모드 변경
            if self.mode == TRAFFIC_WAIT and self.traffic_green:
                self.mode = RUBBERCONE_DRIVE

            elif self.mode == RUBBERCONE_DRIVE and self.end_flag == 1:
                self.mode = RUBBERCONE_END
                self.rubbercone_end_time = now

            elif self.mode == RUBBERCONE_END and elapsed > self.into_lane_timer:
                self.mode = LANE_DRIVE

#            elif self.mode == LANE_DRIVE and self.object_dist != 0: # object_dist need reset
#                self.mode = OBSTACLE_APPROACH
            # elif self.mode == LANE_DRIVE:
            #     cond_exists  = self.obj_exists >= 0.9
            #     cond_dist    = self.object_dist < 1.5
            #     cond_cluster = self.obj_cluster < 15.0

            #     if cond_exists and cond_dist and cond_cluster:
            #         self.cond_count += 1
            #         if self.cond_count >= self.cond_threshold:
            #             self.mode = OBSTACLE_APPROACH
            #             self.get_logger().info("장애물 접근 모드로 변경")
            #             self.cond_count = 0  # 조건 달성 후 초기화
            #     else:
            #         self.cond_count = 0  # 조건 끊기면 다시 0
            elif self.mode == LANE_DRIVE:
                # YOLO 박스 넓이 조건
                cond_box = self.box_size >= 10000.0
                if cond_box:
                    self.mode = OBSTACLE_APPROACH
                    self.get_logger().info("장애물 접근 모드로 변경 (box_size 조건)")

            elif self.mode == OBSTACLE_APPROACH: # object_info is detected
                if self.object_dist < 1.3:
                    if self.lane == 0:  # same side 
                        self.lane = 1 # change lane
                    else:
                        self.lane = 0
                    self.mode = CHANGE_LANE
                    self.get_logger().info("차선 변경 모드로 변경")
                elif self.object_dist > 1.3 or self.obj_exists == 0:   # different side
                    self.mode = LANE_DRIVE
                    self.get_logger().info("장애물 차량이 아니였나봄")

            elif self.mode == CHANGE_LANE and self.is_change_end():
                self.mode = LANE_DRIVE
                self.get_logger().info("차선 주행 모드로 변경")
            
        # 오프셋 선택
        offset = self.rubbercone_offset if self.mode == RUBBERCONE_DRIVE else self.lane_offset

        # Controller 업데이트
        self.controller.update(self.mode, offset, self.object_dist)
        angle = self.controller.get_angle()
        speed = self.controller.get_speed()

        now = self.get_clock().now()

        if (self.lane_drive_started == False):
            self.lane_drive_started = True
            self.lane_drive_start_time = now   # 추가

        if (self.mode == 0):
            self.lane_drive_started = False

        # LANE_DRIVE 모드 진입 후 2초간 속도 제한
        if self.mode == LANE_DRIVE and self.lane_drive_start_time is not None:
            elapsed_lane = (now - self.lane_drive_start_time).nanoseconds / 1e9
            if elapsed_lane < 8.0:
                speed = 10.0

        # 모터 제어 메시지 퍼블리시
        motor_msg = Float32MultiArray()
        motor_msg.data = [float(angle), float(speed)]
        self.motor_pub.publish(motor_msg)

        # 모드 정보 퍼블리시
        mode_msg = Int32MultiArray()
        mode_msg.data = [self.mode, self.lane]
        self.mode_pub.publish(mode_msg)

        # log: 화면에 상태 텍스트 그리기
        # 1) 빈 화면 초기화
        log_img = np.zeros((300, 600, 3), dtype=np.uint8)
        # self.get_logger().info(f"Mode++ -> {self.mode}    {angle}  {speed:.1f}    Offset: {offset}    Lane: {self.lane}    Object Info: {self.object_info}    Object Dist: {self.object_dist}")
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
            now = time.time()

            # 쿨다운 중이면 False
            if now - self.last_change_time < 10:
                if now - self.last_log_time > 1.0:  # 1초에 한 번만 로그
                    remaining = 5 - (now - self.last_change_time)
                    self.get_logger().info(f"타이머 작동중... 남은 시간: {remaining:.1f}초")
                    self.last_log_time = now
                return False
            else: # 조건 충족하면 True + 쿨다운 시작
                self.last_change_time = now
                self.get_logger().info("타이머 작동 끝")
                return True

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()