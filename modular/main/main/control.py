# ─────────────────────────────────────────────────────────────────────────────
# control.py
#
# 역할: 모드별 조향각 및 속도를 계산하는 컨트롤러
#
# 모드별 제어 방식:
#   TRAFFIC_WAIT    (0): 정지
#   RUBBERCONE_DRIVE(1): PD 제어 조향 + 고정 속도
#   RUBBERCONE_END  (2): 고정 각도/속도 (차선 진입 조향)
#   LANE_DRIVE      (3): PD 제어 조향 + 속도 감속 로직
#   BEFORE          (4): 차선 주행 조향 + 장애물 접근 감속
#   CHANGE_LANE     (5): PD 제어 조향 + 속도 감속 로직
# ─────────────────────────────────────────────────────────────────────────────

from collections import namedtuple

# ── 주행 모드 상수 ───────────────────────────────────────────────────────────
TRAFFIC_WAIT     = 0  # 신호 대기: 정지
RUBBERCONE_DRIVE = 1  # 라바콘 주행
RUBBERCONE_END   = 2  # 라바콘 종료 후 차선 진입
LANE_DRIVE       = 3  # 차선 주행
BEFORE           = 4  # 장애물 접근 대기
CHANGE_LANE      = 5  # 차선 변경

# ─────────────────────────────────────────────────────────────────────────────
# 파라미터 설정 구역
# 아래 값들을 수정하여 주행 특성을 조정한다.
# ─────────────────────────────────────────────────────────────────────────────

# PD 제어 파라미터: mode → (kp, kd, alpha)
#   kp    : 비례 이득 (오프셋에 비례한 조향)
#   kd    : 미분 이득 (오프셋 변화율에 비례한 감쇠)
#   alpha : 비선형 보정 계수 (0이면 선형 PD)
PD_PARAMS = {
    RUBBERCONE_DRIVE: (1.1,   0.0, 0.0),
    LANE_DRIVE:       (0.145, 0.3, 0.0),
    CHANGE_LANE:      (0.145, 0.3, 0.0),
}

# 속도 제어 파라미터: mode → (max_speed, min_speed, scale_factor)
#   max_speed    : 최대 속도
#   min_speed    : 최소 속도 (조향각이 커도 이 속도 아래로 떨어지지 않음)
#   scale_factor : |조향각| × scale_factor 만큼 최대 속도에서 감속
SPEED_PARAMS = {
    RUBBERCONE_DRIVE: (13.0, 13.0, 0.1),
    LANE_DRIVE:       (31.0, 12.0, 0.5),
    CHANGE_LANE:      (31.0, 12.0, 0.5),
}

# 라바콘 종료 직후 고정 파라미터 (차선 진입 조향)
RUBBERCONE_END_PARAMS = {
    'angle': -31.0,  # 차선 진입 방향 고정 조향각
    'speed':  15.0,  # 차선 진입 속도
}

# 장애물 접근 모드(BEFORE) 파라미터
BEFORE_PARAMS = {
    'kp':              0.15,  # PI 비례 이득
    'ki':              0.1,   # PI 적분 이득
    'target_distance': 60,    # 목표 접근 거리
    'base_speed':      20,    # 기본 속도
}

# ─────────────────────────────────────────────────────────────────────────────

# 내부용 파라미터 구조체 정의
SpeedParams = namedtuple('SpeedParams', ['max_speed', 'min_speed', 'scale_factor'])
PDParams    = namedtuple('PDParams',    ['kp', 'kd', 'alpha'])


class Controller:
    def __init__(self, node):
        """
        Controller 초기화
        딕셔너리로 정의된 파라미터를 namedtuple로 변환하고 상태를 초기화한다.
        """
        # 제어 출력 상태
        self.angle             = 0.0  # 현재 조향각
        self.speed             = 0.0  # 현재 속도
        # 내부 제어 상태
        self.prev_offset       = 0.0  # 이전 오프셋 (PD 미분항 계산용)
        self.obstacle_integral = 0.0  # 장애물 PI 적분 누적값
        self.prev_mode         = TRAFFIC_WAIT

        # PD / 속도 파라미터를 namedtuple로 변환
        self.pd_params = {
            mode: PDParams(*vals) for mode, vals in PD_PARAMS.items()
        }
        self.speed_params = {
            mode: SpeedParams(*vals) for mode, vals in SPEED_PARAMS.items()
        }

        # 라바콘 종료 파라미터
        self.rubbercone_end_angle = RUBBERCONE_END_PARAMS['angle']
        self.rubbercone_end_speed = RUBBERCONE_END_PARAMS['speed']

        # 장애물 접근 파라미터
        self.ob_kp              = BEFORE_PARAMS['kp']
        self.ob_ki              = BEFORE_PARAMS['ki']
        self.ob_target_distance = BEFORE_PARAMS['target_distance']
        self.ob_base_speed      = BEFORE_PARAMS['base_speed']

    def update(self, mode: int, offset: int, obstacle_dist: float):
        """
        메인 업데이트: 모드에 따라 조향각과 속도를 계산한다.
        모드가 바뀌면 내부 상태를 리셋한다.
        """
        # 모드 전환 감지 → 내부 상태 초기화
        if mode != self.prev_mode:
            self.reset()
            self.prev_mode = mode

        if mode == TRAFFIC_WAIT:
            # 신호 대기: 완전 정지
            self.angle, self.speed = 0.0, 0.0

        elif mode in (RUBBERCONE_DRIVE, LANE_DRIVE):
            # PD 조향 + 조향각 기반 속도 감속
            self.angle = self._compute_steering_pd(mode, offset)
            params     = self.speed_params.get(mode)
            self.speed = self._compute_speed_from_angle(mode, self.angle, params) if params else 0.5

        elif mode == RUBBERCONE_END:
            # 라바콘 종료: 고정 각도/속도로 차선 진입
            self.angle = self.rubbercone_end_angle
            self.speed = self.rubbercone_end_speed

        elif mode == BEFORE:
            # 장애물 접근: 차선 주행 조향 + 감속 (-2 보정)
            self.angle = self._compute_steering_pd(LANE_DRIVE, offset)
            params     = self.speed_params.get(LANE_DRIVE)
            self.speed = self._compute_speed_from_angle(mode, self.angle, params) if params else 0.5

        elif mode == CHANGE_LANE:
            # 차선 변경: PD 조향 + 속도 감속
            self.angle = self._compute_steering_pd(mode, offset)
            params     = self.speed_params.get(mode)
            self.speed = self._compute_speed_from_angle(mode, self.angle, params) if params else 0.5

        else:
            # 정의되지 않은 모드: 안전 정지
            self.angle, self.speed = 0.0, 0.0

    def _compute_steering_pd(self, mode: int, offset: int) -> float:
        """
        PD 제어로 조향각을 계산한다.
          error         : 현재 오프셋 (차선 중심 기준 픽셀 편차)
          diff          : 오프셋 변화량 (미분항)
          effective_kp  : alpha에 의한 비선형 보정 적용 kp
        """
        params = self.pd_params.get(mode)
        if not params:
            return 0.0

        error = float(offset)
        diff  = error - self.prev_offset
        self.prev_offset = error

        # alpha > 0이면 오프셋이 클수록 kp를 증폭 (비선형 제어)
        effective_kp = params.kp * (1.0 + params.alpha * abs(error))
        return effective_kp * error + params.kd * diff

    def _compute_speed_from_angle(self, mode: int, angle: float,
                                   params: SpeedParams) -> float:
        """
        조향각에 따른 속도를 계산한다.
        각도가 클수록 속도가 감소하며, min_speed 이하로 떨어지지 않는다.
        BEFORE 모드에서는 추가로 2 감속한다.
        """
        speed = params.max_speed - abs(angle) * params.scale_factor
        print("speed: ", speed)
        base = max(params.min_speed, speed)
        return base - 2 if mode == BEFORE else base

    def reset(self):
        """내부 제어 상태 초기화 (모드 전환 시 호출)"""
        self.prev_offset       = 0.0
        self.obstacle_integral = 0.0

    def get_angle(self) -> float:
        """현재 계산된 조향각 반환"""
        return self.angle

    def get_speed(self) -> float:
        """현재 계산된 속도 반환"""
        return self.speed
