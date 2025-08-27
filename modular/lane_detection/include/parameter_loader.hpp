#pragma once
#include <string>

enum class LaneMode {
    LANE_ONE,
    LANE_TWO
};

struct Config {
    int yellow_hls_min_h;
    int yellow_hls_max_h;
    int yellow_hls_min_l;
    int yellow_hls_max_l;
    int yellow_hls_min_s;
    int yellow_hls_max_s;
    int yellow_hsv_min_h;
    int yellow_hsv_max_h;
    int yellow_hsv_min_s;
    int yellow_hsv_max_s;
    int yellow_hsv_min_v;
    int yellow_hsv_max_v;
    int yellow_ycrcb_min_y;
    int yellow_ycrcb_max_y;
    int yellow_ycrcb_min_cr;
    int yellow_ycrcb_max_cr;
    int yellow_ycrcb_min_cb;
    int yellow_ycrcb_max_cb;
    LaneMode lane_mode;
    int frame_width;
    int frame_height;
    float roi_top_width_coefficient;
    float roi_bottom_width_coefficient;
    float roi_top_y_coefficient;
    float roi_bottom_y_coefficient;
    float ref_hist_sigma_ratio;
    float ref_hist_min_weight;
    int sliding_window_num_windows;
    int sliding_window_margin;
    size_t sliding_window_minpix;
    int corridor_width;
    int gaussian_blur_kernel_size;
    int canny_yellow_high_threshold;
    int canny_yellow_low_threshold;
    int kernel_yellow_closing_size;
    int kernel_yellow_opening_size;
    float center_reference_lane_one;
    float center_reference_lane_two;
    int horizontal_noise_width;
    int horizontal_noise_band_h;
    int horizontal_noise_extra_pad;
    float horizontal_band_corridor_ratio_thresh;
    int vertical_noise_band_w;
    int vertical_noise_min_pixels;
    float vertical_noise_peak_ratio;
    int vertical_noise_extra_pad_half_width;
    bool vertical_noise_peak_use_corridor;
    float lane_change_tol_straight;
    float lane_change_tol_curve;
    float lane_change_tol_change;
    int lane_change_streak_need;
    float lane_change_m_split;
    float lane_ref_transition_duration_sec;
    bool debug_view;
    bool change_ref_smoothly;
};
LaneMode lane_mode_from_string(const std::string& mode_str);
Config load_config(const std::string& path);