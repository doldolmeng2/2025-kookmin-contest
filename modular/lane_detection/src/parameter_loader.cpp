#include <fstream>
#include <nlohmann/json.hpp>
#include "parameter_loader.hpp"
using json = nlohmann::json;

LaneMode lane_mode_from_string(const std::string& mode_str) {
    if (mode_str == "LANE_ONE") return LaneMode::LANE_ONE;
    else if (mode_str == "LANE_TWO") return LaneMode::LANE_TWO;
    else throw std::runtime_error("Invalid lane_mode value in config: " + mode_str);
}

Config load_config(const std::string& path) {
    std::ifstream file(path);
    json j;
    file >> j;

    Config config;
    config.yellow_min_h = j["yellow_min_h"];
    config.yellow_max_h = j["yellow_max_h"];
    config.yellow_min_s = j["yellow_min_s"];
    config.yellow_max_s = j["yellow_max_s"];
    config.yellow_min_v = j["yellow_min_v"];
    config.yellow_max_v = j["yellow_max_v"];
    config.white_min_h = j["white_min_h"];
    config.white_max_h = j["white_max_h"];
    config.white_min_s = j["white_min_s"];
    config.white_max_s = j["white_max_s"];
    config.white_min_v = j["white_min_v"];
    config.white_max_v = j["white_max_v"];
    config.lane_mode = lane_mode_from_string(j["lane_mode"]);
    config.frame_width = j["frame_width"];
    config.frame_height = j["frame_height"];
    config.roi_height_coefficient = j["roi_height_coefficient"];
    config.roi_top_width_coefficient = j["roi_top_width_coefficient"];
    config.roi_bottom_width_coefficient = j["roi_bottom_width_coefficient"];
    config.sliding_window_num_windows = j["sliding_window_num_windows"];
    config.sliding_window_margin = j["sliding_window_margin"];
    config.sliding_window_minpix = j["sliding_window_minpix"];
    config.gaussian_blur_kernel_size = j["gaussian_blur_kernel_size"];
    config.canny_yellow_high_threshold = j["canny_yellow_high_threshold"];
    config.canny_yellow_low_threshold = j["canny_yellow_low_threshold"];
    config.canny_white_high_threshold = j["canny_white_high_threshold"];
    config.canny_white_low_threshold = j["canny_white_low_threshold"];
    config.kernel_yellow_closing_size = j["kernel_yellow_closing_size"];
    config.kernel_yellow_opening_size = j["kernel_yellow_opening_size"];
    config.kernel_white_closing_size = j["kernel_white_closing_size"];
    config.kernel_white_opening_size = j["kernel_white_opening_size"];
    config.center_reference_lane_one = j["center_reference_lane_one"];
    config.center_reference_lane_two = j["center_reference_lane_two"];
    return config;
}
