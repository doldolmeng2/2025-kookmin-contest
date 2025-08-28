import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int16, Bool
from std_msgs.msg import Float32MultiArray
from main.control import Controller
import cv2
import numpy as np
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# 모드 상수
TRAFFIC_WAIT = 0
RUBBERCONE_DRIVE = 1
RUBBERCONE_END = 2
LANE_DRIVE = 3
BEFORE = 4
CHANGE_LANE = 5

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')

        # ---- Parameter / State ----
        self.declare_parameter('mode', TRAFFIC_WAIT)
        self.mode = self.get_parameter('mode').value
        self.last_change_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()
        self.cond_count = 0
        self.cond_threshold = 2  # 몇 프레임 이상 유지할지

        # Controller
        self.controller = Controller(self)

        # ---- Pub/Sub ----

        qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE   # latch 안 함
        )
        
        self.motor_pub = self.create_publisher(Float32MultiArray, 'xycar_motor', 10)
        self.mode_pub  = self.create_publisher(Int32MultiArray,  'mode_info',   qos_sensor)

        self.create_subscription(Int32MultiArray, 'rubbercone_info',  self.rubbercone_callback, qos_sensor)
        self.create_subscription(Int16,           'lane_offset',      self.lane_offset_callback, qos_sensor)
        self.create_subscription(Float32MultiArray,'object_info',     self.object_info_callback, qos_sensor)
        self.create_subscription(Bool,            'traffic_detection',self.traffic_callback,     qos_sensor)
        self.create_subscription(Joy,             'joy',              self.joy_callback,         10)
        self.create_subscription(Int32MultiArray, 'xycar_ultrasonic', self.ultrasonic_callback, 10)
        # Xbox 버튼 디바운스
        self.prev_x = 0
        self.prev_b = 0

        # ---- Variables ----
        self.test_mode = False
        self.lane = 1  # 0=Lane1, 1=Lane2
        self.rubbercone_offset = 0
        self.end_flag = 0
        self.lane_offset = 0
        self.object_info = -1
        self.object_dist = 0.0
        self.traffic_green = False
        self.rubbercone_end_time = None
        self.into_lane_timer = 1.4
        self.lane_drive_started = False
        self.lane_drive_start_time = None
        self.now_speed = 0

        # ---- object_info 기본값 (10필드용) ----
        self.obj_exists   = 0.0
        self.object_dist  = float('inf')  # m
        self.obj_angle    = 0.0           # rad
        self.obj_span     = 0.0           # rad
        self.obj_cluster  = 0.0
        self.box_size     = 0.0           # px^2
        self.box_cx       = float('nan')  # px
        self.box_cy       = float('nan')  # px
        self.box_dx       = float('nan')  # px
        self.car_lane     = -1            # -1=미정, 1=L1(왼쪽), 2=L2(오른쪽), 0=중앙
        self.avoid_cnt = 0
        # 20 ms timer (~50 Hz)
        self.create_timer(0.02, self.control_cycle)

    # ---------- Subscriptions ----------

    def joy_callback(self, msg: Joy):
        """X: mode--, B: mode++ (rising edge)"""
        x = msg.buttons[2]
        b = msg.buttons[1]
        if x == 1 and self.prev_x == 0:
            self.end_flag = 0
            self.rubbercone_end_time = None
            self.mode = max(TRAFFIC_WAIT, self.mode - 1)
            self.get_logger().info(f"Mode-- -> {self.mode}")
        if b == 1 and self.prev_b == 0:
            self.mode = min(CHANGE_LANE, self.mode + 1)
            self.get_logger().info(f"Mode++ -> {self.mode}")
        self.prev_x = x
        self.prev_b = b

    def rubbercone_callback(self, msg: Int32MultiArray):
        if len(msg.data) >= 2:
            self.rubbercone_offset = msg.data[0]
            self.end_flag          = msg.data[1]

    def lane_offset_callback(self, msg: Int16):
        self.lane_offset = msg.data

    def object_info_callback(self, msg: Float32MultiArray):
        # /object_info (10개):
        # [exists, min_dist, angle, span, cluster_size, box_size, box_cx, box_cy, dx, car_lane]
        d = msg.data
        self.obj_exists   = float(d[0])   # 0/1
        self.object_dist  = float(d[1])   # m
        self.obj_angle    = float(d[2])   # rad
        self.obj_span     = float(d[3])   # rad
        self.obj_cluster  = float(d[4])   # count
        self.box_size     = float(d[5])   # px^2
        self.box_cx       = float(d[6])   # px
        self.box_cy       = float(d[7])   # px
        self.box_dx       = float(d[8])   # px
        self.car_lane     = int(d[9])     # 1=L1, 2=L2, 0=C, -1=미정(발행 안되면 상위에서 유지)

    def traffic_callback(self, msg: Bool):
        self.traffic_green = msg.data
    
    def ultrasonic_callback(self, msg: Int32MultiArray):
        data = msg.data   # list[int] 형태
        if len(data) > 5:
            self.left = data[0]
            self.right  = data[4]
        # self.get_logger().info(f"left={self.left}, right={self.right}")

    # ---------- Control Loop ----------

    def control_cycle(self):
        now = self.get_clock().now()
        if self.rubbercone_end_time is not None:
            elapsed = (now - self.rubbercone_end_time).nanoseconds / 1e9
        else:
            elapsed = 0.0

        if not self.test_mode:
            if self.mode == TRAFFIC_WAIT and self.traffic_green:
                self.mode = RUBBERCONE_DRIVE
                self.get_logger().info("초록불 감지 -> 라바콘 모드로 변경")

            elif self.mode == RUBBERCONE_DRIVE and self.end_flag == 1:
                self.mode = RUBBERCONE_END
                self.rubbercone_end_time = now
                self.get_logger().info("라바콘 종료 차선 진입")

            elif self.mode == RUBBERCONE_END and elapsed > self.into_lane_timer:
                self.mode = BEFORE

            elif self.mode == BEFORE:
                if self.is_pass_comp():
                    self.mode = LANE_DRIVE
                    self.get_logger().info("추월 전 상태")
            elif self.mode == LANE_DRIVE:
                if self.box_size > 1900:
                    self.mode = CHANGE_LANE
                    self.lane = 1 - self.lane
                    self.get_logger().info("객체 박스 감지, 차선 변경 모드 전환")
            elif self.mode == CHANGE_LANE and self.is_change_end():
                self.mode = BEFORE
                self.get_logger().info("차선 변경 완료")
        

        # 오프셋 선택
        offset = self.rubbercone_offset if self.mode == RUBBERCONE_DRIVE else self.lane_offset

        # Controller 업데이트
        self.controller.update(self.mode, offset, self.object_dist)
        angle = self.controller.get_angle()
        speed = self.controller.get_speed()
        if self.now_speed < speed:
            self.now_speed += 0.1
        else:
            self.now_speed = speed

        now = self.get_clock().now()
        if not self.lane_drive_started:
            self.lane_drive_started = True
            self.lane_drive_start_time = now
        if self.mode == TRAFFIC_WAIT:
            self.lane_drive_started = False

        # LANE_DRIVE 진입 후 8초간 속도 제한
        if self.mode == LANE_DRIVE and self.lane_drive_start_time is not None:
            elapsed_lane = (now - self.lane_drive_start_time).nanoseconds / 1e9
            if elapsed_lane < 3.0:
                speed = 5.0

        # 모터 퍼블리시
        motor_msg = Float32MultiArray()
        motor_msg.data = [float(angle), float(self.now_speed)]
        self.motor_pub.publish(motor_msg)

        # 모드 퍼블리시
        mode_msg = Int32MultiArray()
        mode_msg.data = [self.mode, self.lane]
        self.mode_pub.publish(mode_msg)

        # ---- 간단 상태 화면(Log) ----
        log_img = np.zeros((320, 640, 3), dtype=np.uint8)
        mode_map = {
            TRAFFIC_WAIT:      'TRAFFIC_WAIT',
            RUBBERCONE_DRIVE:  'RUBBERCONE_DRIVE',
            RUBBERCONE_END:    'RUBBERCONE_END',
            LANE_DRIVE:        'LANE_DRIVE',
            BEFORE:            'BEFORE',
            CHANGE_LANE:       'CHANGE_LANE',
        }
        mode_str    = f"Mode: {mode_map.get(self.mode, 'UNKNOWN')}"
        lane_str    = f"{'Lane 1' if self.lane==0 else 'Lane 2'}"
        endflag_str = 'Rubber End' if self.end_flag==1 else 'Rubber Not End'
        offset_str  = f"Offset: {offset}"
        objdist_str = f"Object dist: {self.object_dist:.2f} m"
        angle_str   = f"Angle: {angle:.1f}"
        speed_str   = f"Speed: {speed:.1f}"
        box_str     = f"Box: {self.box_size:.0f}px^2  car_lane={self.car_lane}"

        y0, dy = 30, 30
        for i, text in enumerate([mode_str, angle_str, speed_str, offset_str,
                                  lane_str, endflag_str, objdist_str, box_str]):
            cv2.putText(log_img, text, (10, y0 + i*dy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.imshow('Status', log_img)
        cv2.waitKey(1)

    def is_change_end(self):
        return True
    
    def is_pass_comp(self):
        if self.lane == 0 and self.right < 25:
            self.avoid_cnt += 1
            print("왼쪽 감지  카운트 :", self.avoid_cnt)
        elif self.lane == 1 and self.left < 25:
            self.avoid_cnt += 1
            print("오른쪽 감지  카운트 :", self.avoid_cnt)
        if self.avoid_cnt > 10:
            self.avoid_cnt = 0
            return True
        else:
            return False
        # 어떤 차선일때 반대쪽 초음파 20cm 이내 10번이상 감지되면

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
