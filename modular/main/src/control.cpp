#include "main/control.hpp"
#include <fstream>
#include <iostream>
#include <cmath>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

Controller::Controller() 
    : angle_(0.0), speed_(0.0), prev_offset_(0.0), obstacle_integral_(0.0), prev_mode_(0) {}

// 메인 제어 업데이트 함수
void Controller::update(int mode, int offset, double obstacle_dist) {
    if (prev_mode_ != mode) {
        reset();
        prev_mode_ = mode;
    }
    switch (mode) {
        case TRAFFIC_WAIT:
            // 신호 대기 모드: 정지
            angle_ = 0.0;
            speed_ = 0.0;
            break;

        case RUBBERCONE_DRIVE:
        case LANE_DRIVE:
        case CHANGE_LANE: {
            // 일반 주행 모드: 조향은 PD 제어, 속도는 angle 기반 제어
            angle_ = computeSteeringPD(mode, offset);
            auto it = speed_params_.find(mode);
            if (it != speed_params_.end()) {
                speed_ = computeSpeedFromAngle(angle_, it->second);
            } else {
                speed_ = 0.5; // 파라미터 미존재 시 기본값
            }
            break;
        }

        // 라바콘 종료 시: 임시 고정값
        case RUBBERCONE_END:
            angle_ = rubbercone_end_params_.angle;
            speed_ = rubbercone_end_params_.speed;
            break;

        case OBSTACLE_APPROACH:
            // 앞차 접근 모드: 조향은 lane_drive와 동일, 속도는 거리 기반 PI 제어
            angle_ = computeSteeringPD(LANE_DRIVE, offset);
            if (obstacle_dist > 0.0) {
                speed_ = computeObstacleSpeed(obstacle_dist);
            } else {
                speed_ = 0.0; // 장애물 거리 데이터 없으면 안전을 위해 정지
            }
            break;

        default:
            // 정의되지 않은 모드: 안전 정지
            angle_ = 0.0;
            speed_ = 0.0;
    }
}

// JSON 파라미터 로드
bool Controller::loadParameters(const std::string& json_path) {
    std::ifstream file(json_path);
    if (!file.is_open()) {
        std::cerr << "[ERROR] 파라미터 파일을 열 수 없습니다: " << json_path << "\n";
        return false;
    }

    json config;
    file >> config;

    try {
        // PD 파라미터 로드
        for (auto& [key, value] : config["PD"].items()) {
            int mode = std::stoi(key);
            pd_params_[mode] = {value["kp"], value["kd"], value.value("alpha", 0.0)};
        }

        // 일반 속도 제어 파라미터 로드
        for (auto& [key, value] : config["SPEED"].items()) {
            int mode = std::stoi(key);
            speed_params_[mode] = {value["max_speed"], value["min_speed"], value["scale_factor"]};
        }

        // 장애물 접근 모드 파라미터 로드
        auto obs = config["OBSTACLE_APPROACH"];
        obstacle_params_.kp = obs["kp"];
        obstacle_params_.ki = obs["ki"];
        obstacle_params_.target_distance = obs["target_distance"];
        obstacle_params_.base_speed = obs["base_speed"];

        auto rce = config["RUBBERCONE_END"];
        rubbercone_end_params_.angle = rce["angle"];
        rubbercone_end_params_.speed = rce["speed"];

    } catch (std::exception& e) {
        std::cerr << "[ERROR] JSON 파싱 실패: " << e.what() << "\n";
        return false;
    }

    return true;
}

// PD 제어를 이용한 조향각 계산
double Controller::computeSteeringPD(int mode, int offset) {
    auto it = pd_params_.find(mode);
    if (it == pd_params_.end()) return 0.0;

    double error = static_cast<double>(offset);
    double diff = error - prev_offset_;
    prev_offset_ = error;

    // 비선형 P 게인 적용
    double effective_kp = it->second.kp * (1.0 + it->second.alpha * std::abs(error));

    return effective_kp * error + it->second.kd * diff;
}

// 일반 주행 모드에서 angle 기반 속도 계산
double Controller::computeSpeedFromAngle(double angle, const SpeedParams& params) {
    // 조향각이 클수록 속도는 낮아진다
    double speed = params.max_speed - std::abs(angle) * params.scale_factor;

    // 속도 제한
    if (speed < params.min_speed) speed = params.min_speed;
    return speed;
}

// 장애물 접근 모드에서 PI 제어를 이용한 속도 계산
double Controller::computeObstacleSpeed(double current_dist) {
    // 오차 계산
    double error = current_dist - obstacle_params_.target_distance;
    obstacle_integral_ += error; // 적분 더하기

    double adjustment = obstacle_params_.kp * error + obstacle_params_.ki * obstacle_integral_;
    // 최대 속도 클리핑을 위해 차선 주행 속도를 최대 속도로 설정
    double lane_speed_limit = computeSpeedFromAngle(angle_, speed_params_[LANE_DRIVE]);
    // base_speed는 JSON에서 로드 가능 (예: "base_speed": 0.5)
    double speed = obstacle_params_.base_speed + adjustment;

    if (speed < 0) speed = 0.0;
    if (speed > lane_speed_limit) speed = lane_speed_limit;

    return speed;
}

void Controller::reset() {
    obstacle_integral_ = 0.0;
}

// 조향각 반환
double Controller::getAngle() const { return angle_; }

// 속도 반환
double Controller::getSpeed() const { return speed_; }
