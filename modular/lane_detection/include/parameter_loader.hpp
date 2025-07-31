#pragma once
#include <string>

enum class LaneMode {
    ONE_LANE,
    TWO_LANE
};

struct Config {
    int yellow_min_h;
    int yellow_max_h;
    int yellow_min_s;
    int yellow_max_s;
    int yellow_min_v;
    int yellow_max_v;
    int white_min_h;
    int white_max_h;
    int white_min_s;
    int white_max_s;
    int white_min_v;
    int white_max_v;
    LaneMode lane_mode;
    int frame_width;
    int frame_height;
    float roi_height_coefficient;
    float roi_top_width_coefficient;
    float roi_bottom_width_coefficient;
    int sliding_window_num_windows;
    int sliding_window_margin;
    size_t sliding_window_minpix;
    int gaussian_blur_kernel_size;
    int canny_high_threshold;
    int canny_low_threshold;
    int kernel_yellow_closing_size;
    int kernel_yellow_opening_size;
    int kernel_white_closing_size;
    int kernel_white_opening_size;
};

LaneMode lane_mode_from_string(const std::string& mode_str);
Config load_config(const std::string& path);