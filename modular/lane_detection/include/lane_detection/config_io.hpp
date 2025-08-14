#pragma once
#include <string>
#include "config_types.hpp"

namespace lane {
namespace config {
namespace io {

/**
 * @brief JSON 파일에서 LaneConfig 로드
 * @param path  JSON 파일 경로
 * @param out   결과
 * @param error 실패 시 에러 메시지 (옵션)
 * @return true on success
 */
bool LoadFromFile(const std::string& path, LaneConfig& out, std::string* error = nullptr);

/**
 * @brief LaneConfig를 JSON 파일로 저장
 * @param path  JSON 파일 경로
 * @param cfg   입력 설정
 * @param error 실패 시 에러 메시지 (옵션)
 * @return true on success
 */
bool SaveToFile(const std::string& path, const LaneConfig& cfg, std::string* error = nullptr);

/**
 * @brief 파일이 있으면 로드, 없거나 실패하면 기본값 반환
 * @param path               JSON 파일 경로
 * @param loaded_from_file   실제 파일에서 읽었는지 여부 (옵션)
 * @param error              실패 시 에러 메시지 (옵션)
 */
LaneConfig LoadOrDefault(const std::string& path,
                         bool* loaded_from_file = nullptr,
                         std::string* error = nullptr);

} // namespace io
} // namespace config
} // namespace lane
