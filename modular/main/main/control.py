# control.py

from collections import namedtuple

# 모드 상수 정의
# 각 주행 모드를 숫자 상수로 지정
TRAFFIC_WAIT      = 0  # 신호 대기 모드
RUBBERCONE_DRIVE  = 1  # 라바콘 주행 모드
RUBBERCONE_END    = 2  # 라바콘 종료 모드
LANE_DRIVE        = 3  # 차선 주행 모드
BEFORE = 4  # 장애물 접근 모드
CHANGE_LANE       = 5  # 차선 변경 모드

# ——————————————————————————————————————————————————————————————
# 파라미터 정의 구역
# 여기서 제어 알고리즘에 사용될 파라미터를 조정하세요.

# PD 제어 파라미터: mode → (kp, kd, alpha)
# kp: 비례 이득, kd: 미분 이득, alpha: 비선형 보정 계수
PD_PARAMS = {
    RUBBERCONE_DRIVE: (1.1, 0.0, 0.0),
    LANE_DRIVE:       (0.145, 0.3, 0.0),
    CHANGE_LANE:      (0.145, 0.3, 0.0),
}

# 속도 제어 파라미터: mode → (max_speed, min_speed, scale_factor)
# max_speed: 최대 속도, min_speed: 최소 속도, scale_factor: 조향각 스케일 계수
SPEED_PARAMS = {
    RUBBERCONE_DRIVE: (13.0, 13.0, 0.1),
    LANE_DRIVE:       (31.0, 12.0, 0.5),
    CHANGE_LANE:      (31.0, 12.0, 0.5),
}

# 라바콘 종료 시 고정 파라미터
# angle: 종료 직후 조향 각도, speed: 종료 직후 속도
RUBBERCONE_END_PARAMS = {
    'angle': -31.0,
    'speed': 15.0,
}

# 장애물 접근 모드 파라미터
# kp, ki: PI 제어 이득, target_distance: 목표 거리, base_speed: 기본 속도
BEFORE_PARAMS = {
    'kp':              0.15,
    'ki':              0.1,
    'target_distance': 60,
    'base_speed':      20,
}
# ——————————————————————————————————————————————————————————————

# 내부용 namedtuple 정의
# SpeedParams: 속도 제어 파라미터 구조체
# PDParams:   PD 제어 파라미터 구조체
SpeedParams = namedtuple('SpeedParams', ['max_speed', 'min_speed', 'scale_factor'])
PDParams    = namedtuple('PDParams',    ['kp', 'kd', 'alpha'])

class Controller:
    def __init__(self, node):
        """
        Controller 객체 초기화
        - 내부 상태 초기화
        - 파라미터 딕셔너리를 namedtuple로 변환
        """
        # 제어 상태 변수
        self.angle             = 0.0   # 현재 조향 각도
        self.speed             = 0.0   # 현재 속도
        self.prev_offset       = 0.0   # 이전 오프셋(차선 위치)
        self.obstacle_integral = 0.0   # 장애물 PI 제어 적분 값
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

        # 장애물 접근 파라미터
        self.ob_kp              = BEFORE_PARAMS['kp']
        self.ob_ki              = BEFORE_PARAMS['ki']
        self.ob_target_distance = BEFORE_PARAMS['target_distance']
        self.ob_base_speed      = BEFORE_PARAMS['base_speed']

    def update(self, mode, offset: int, obstacle_dist: int):
        """
        메인 업데이트 함수
        - 모드 전환 시 내부 상태 리셋
        - 각 모드에 따라 조향과 속도를 계산
        """
        # 모드 변경 감지 및 리셋
        if mode != self.prev_mode:
            self.reset()
            self.prev_mode = mode

        # 모드별 제어 로직
        if mode == TRAFFIC_WAIT:
            # 신호 대기 시 정지
            self.angle, self.speed = 0.0, 0.0

        elif mode in (RUBBERCONE_DRIVE, LANE_DRIVE):
            # PD 제어로 조향 계산 후 속도 제어
            self.angle = self._compute_steering_pd(mode, offset)
            params     = self.speed_params.get(mode)
            self.speed = self._compute_speed_from_angle(mode, self.angle, params) if params else 0.5

        elif mode == RUBBERCONE_END:
            # 라바콘 종료 후 고정 파라미터 사용
            self.angle, self.speed = self.rubbercone_end_angle, self.rubbercone_end_speed

        elif mode == BEFORE:
            # 장애물 접근: 차선 주행 조향 + PI 제어 속도
            self.angle = self._compute_steering_pd(LANE_DRIVE, offset)
            # self.speed = 15
            params     = self.speed_params.get(LANE_DRIVE)
            self.speed = self._compute_speed_from_angle(mode, self.angle, params) if params else 0.5

        elif mode == CHANGE_LANE:
            # 장애물 접근: 차선 주행 조향 + PI 제어 속도
            self.angle = self._compute_steering_pd(mode, offset)
            params     = self.speed_params.get(mode)
            self.speed = self._compute_speed_from_angle(mode, self.angle, params) if params else 0.5

        else:
            # 정의되지 않은 모드에서는 안전 정지
            self.angle, self.speed = 0.0, 0.0

    def _compute_steering_pd(self, mode: int, offset: int) -> float:
        """
        PD 제어를 이용해 조향 각도 계산
        - error: 현재 오프셋
        - diff:  이전 오프셋 차이
        - effective_kp: 비선형 보정 적용된 KP
        """
        params = self.pd_params.get(mode)
        if not params:
            return 0.0

        error = float(offset)
        diff  = error - self.prev_offset
        self.prev_offset = error

        effective_kp = params.kp * (1.0 + params.alpha * abs(error))
        return effective_kp * error + params.kd * diff

    def _compute_speed_from_angle(self, mode: int, angle: float, params: SpeedParams) -> float:
        """
        조향 각도에 따른 속도 계산
        - 각도가 클수록 속도 감소
        - min_speed 이하로 떨어지지 않도록 제한
        """
        speed = params.max_speed - abs(angle) * params.scale_factor
        print("speed: ", speed)
        return max(params.min_speed, speed) - 2 if mode == BEFORE else max(params.min_speed, speed)

    def reset(self):
        """
        내부 제어 변수 초기화
        - 오프셋 적분 및 이전 오프셋 초기화
        """
        self.prev_offset       = 0.0
        self.obstacle_integral = 0.0

    def get_angle(self) -> float:
        """현재 계산된 조향 각도 반환"""
        return self.angle

    def get_speed(self) -> float:
        """현재 계산된 속도 반환"""
        return self.speed
