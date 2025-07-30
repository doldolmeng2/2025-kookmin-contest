#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <string>
#include <unordered_map>

// 모드 정의
// 0: 신호 대기, 1: 라바콘 주행, 2: 라바콘 종료, 3: 차선 주행,
// 4: 앞차 접근(거리 유지), 5: 차선 변경
enum Mode {
    TRAFFIC_WAIT = 0,
    RUBBERCONE_DRIVE = 1,
    RUBBERCONE_END = 2,
    LANE_DRIVE = 3,
    OBSTACLE_APPROACH = 4,
    CHANGE_LANE = 5
};

// PD 제어용 파라미터 구조체
struct PDParams {
    double kp;     // 기본 P 게인
    double kd;     // D 게인
    double alpha;  // 비선형 증폭 계수
};

// 일반 주행 속도 제어용 파라미터 구조체
// angle 값에 따라 속도를 조절할 때 사용
struct SpeedParams {
    double max_speed;    // 최대 속도
    double min_speed;    // 최소 속도
    double scale_factor; // 조향각에 따른 감속 계수
};

// 장애물 접근 모드에서 사용하는 PI 제어 파라미터 구조체
struct ObstacleParams {
    double kp;             // 비례 이득
    double ki;             // 적분 이득
    double target_distance; // 유지하고 싶은 목표 거리
};
struct RubberconeEndParams {
    double angle;
    double speed;
};

class Controller {
public:
    Controller();

    // JSON 파일에서 제어 파라미터를 로드
    bool loadParameters(const std::string& json_path);

    // 제어 업데이트 함수
    // mode: 현재 주행 모드
    // offset: 차선 혹은 라바콘에 대한 오프셋 값
    // obstacle_dist: 앞차와의 거리 (미사용 시 -1.0)
    void update(int mode, int offset, double obstacle_dist = -1.0);

    // angle(조향각)과 speed(속도) 결과 반환 함수
    double getAngle() const;
    double getSpeed() const;

private:
    // 조향각 계산 (PD 제어)
    double computeSteeringPD(int mode, int offset);

    // 일반 주행에서 조향각 기반 속도 계산
    double computeSpeedFromAngle(double angle, const SpeedParams& params);

    // 장애물 접근 모드에서 거리 기반 속도 계산 (PI 제어)
    double computeObstacleSpeed(double current_dist);

    void reset();


    // 내부 상태 변수
    double angle_;                // 현재 조향각
    double speed_;                // 현재 속도
    double prev_offset_;          // 이전 오프셋 값 (PD 제어에서 미분항 계산용)
    double obstacle_integral_;    // 장애물 접근 PI 제어에서 적분항 누적 값
    int prev_mode_;

    // 파라미터 맵
    std::unordered_map<int, PDParams> pd_params_;       // 모드별 PD 파라미터
    std::unordered_map<int, SpeedParams> speed_params_; // 모드별 속도 파라미터
    ObstacleParams obstacle_params_;                    // 장애물 접근 모드 파라미터
    RubberconeEndParams rubbercone_end_params_;
};

#endif
