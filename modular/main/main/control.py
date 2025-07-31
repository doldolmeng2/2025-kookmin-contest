# control.py

from collections import namedtuple

# 모드 상수
TRAFFIC_WAIT      = 0
RUBBERCONE_DRIVE  = 1
RUBBERCONE_END    = 2
LANE_DRIVE        = 3
OBSTACLE_APPROACH = 4
CHANGE_LANE       = 5

# ——————————————————————————————————————————————————————————————
# 파라미터 정의 (여기서 조정)
# PD 제어 파라미터: mode → (kp, kd, alpha)
PD_PARAMS = {
    RUBBERCONE_DRIVE: (1.2, 0.02, 0.01),
    LANE_DRIVE:       (1.0, 0.01, 0.0),
    CHANGE_LANE:      (1.0, 0.01, 0.0),
}

# 속도 제어 파라미터: mode → (max_speed, min_speed, scale_factor)
SPEED_PARAMS = {
    RUBBERCONE_DRIVE: (40.0, 30.0, 0.1),
    LANE_DRIVE:       (50.0, 30.0, 0.1),
    CHANGE_LANE:      (30.0, 20.0, 0.1),
}

# 라바콘 종료 시 고정 파라미터
RUBBERCONE_END_PARAMS = {
    'angle': 0.0,
    'speed': 20.0,
}

# 장애물 접근 모드 파라미터
OBSTACLE_PARAMS = {
    'kp':              0.5,
    'ki':              0.1,
    'target_distance': 60,
    'base_speed':      30,
}
# ——————————————————————————————————————————————————————————————

# 내부용 namedtuple
SpeedParams = namedtuple('SpeedParams', ['max_speed', 'min_speed', 'scale_factor'])
PDParams    = namedtuple('PDParams',    ['kp', 'kd', 'alpha'])

class Controller:
    def __init__(self):
        # 내부 상태 초기화
        self.angle             = 0.0
        self.speed             = 0.0
        self.prev_offset       = 0.0
        self.obstacle_integral = 0.0
        self.prev_mode         = TRAFFIC_WAIT

        # 파라미터 구조체 변환
        self.pd_params = {
            mode: PDParams(*vals)
            for mode, vals in PD_PARAMS.items()
        }
        self.speed_params = {
            mode: SpeedParams(*vals)
            for mode, vals in SPEED_PARAMS.items()
        }
        self.rubbercone_end_angle = RUBBERCONE_END_PARAMS['angle']
        self.rubbercone_end_speed = RUBBERCONE_END_PARAMS['speed']

        self.ob_kp              = OBSTACLE_PARAMS['kp']
        self.ob_ki              = OBSTACLE_PARAMS['ki']
        self.ob_target_distance = OBSTACLE_PARAMS['target_distance']
        self.ob_base_speed      = OBSTACLE_PARAMS['base_speed']

    def update(self, mode: int, offset: float, obstacle_dist: float):
        # 모드 변경 시 상태 리셋
        if mode != self.prev_mode:
            self.reset()
            self.prev_mode = mode

        # 모드별 제어
        if mode == TRAFFIC_WAIT:
            self.angle, self.speed = 0.0, 0.0

        elif mode in (RUBBERCONE_DRIVE, LANE_DRIVE, CHANGE_LANE):
            # PD 조향 + 각도 기반 속도
            self.angle = self._compute_steering_pd(mode, offset)
            params     = self.speed_params.get(mode)
            self.speed = self._compute_speed_from_angle(self.angle, params) if params else 0.5

        elif mode == RUBBERCONE_END:
            self.angle, self.speed = self.rubbercone_end_angle, self.rubbercone_end_speed

        elif mode == OBSTACLE_APPROACH:
            # 차선 주행 조향 + PI 장애물 속도
            self.angle = self._compute_steering_pd(LANE_DRIVE, offset)
            self.speed = self._compute_obstacle_speed(obstacle_dist) if obstacle_dist > 0 else 0.0

        else:
            self.angle, self.speed = 0.0, 0.0

    def _compute_steering_pd(self, mode: int, offset: float) -> float:
        params = self.pd_params.get(mode)
        if not params:
            return 0.0
        error = float(offset)
        diff  = error - self.prev_offset
        self.prev_offset = error

        effective_kp = params.kp * (1.0 + params.alpha * abs(error))
        return effective_kp * error + params.kd * diff

    def _compute_speed_from_angle(self, angle: float, params: SpeedParams) -> float:
        speed = params.max_speed - abs(angle) * params.scale_factor
        return max(params.min_speed, speed)

    def _compute_obstacle_speed(self, current_dist: float) -> float:
        error = current_dist - self.ob_target_distance
        self.obstacle_integral += error
        adjustment = self.ob_kp * error + self.ob_ki * self.obstacle_integral

        # 차선 주행 속도 제한
        max_lane_speed = self._compute_speed_from_angle(self.angle, self.speed_params[LANE_DRIVE])
        speed = self.ob_base_speed + adjustment
        return min(max(speed, 0.0), max_lane_speed)

    def reset(self):
        self.prev_offset       = 0.0
        self.obstacle_integral = 0.0

    def get_angle(self) -> float:
        return self.angle

    def get_speed(self) -> float:
        return self.speed
