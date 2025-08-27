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
    config.yellow_hls_min_h = j["yellow_hls_min_h"];
    config.yellow_hls_max_h = j["yellow_hls_max_h"];
    config.yellow_hls_min_l = j["yellow_hls_min_l"];
    config.yellow_hls_max_l = j["yellow_hls_max_l"];
    config.yellow_hls_min_s = j["yellow_hls_min_s"];
    config.yellow_hls_max_s = j["yellow_hls_max_s"];
    config.yellow_hsv_min_h = j["yellow_hsv_min_h"];
    config.yellow_hsv_max_h = j["yellow_hsv_max_h"];
    config.yellow_hsv_min_s = j["yellow_hsv_min_s"];
    config.yellow_hsv_max_s = j["yellow_hsv_max_s"];
    config.yellow_hsv_min_v = j["yellow_hsv_min_v"];
    config.yellow_hsv_max_v = j["yellow_hsv_max_v"];
    config.yellow_ycrcb_min_y = j["yellow_ycrcb_min_y"];
    config.yellow_ycrcb_max_y = j["yellow_ycrcb_max_y"];
    config.yellow_ycrcb_min_cr = j["yellow_ycrcb_min_cr"];
    config.yellow_ycrcb_max_cr = j["yellow_ycrcb_max_cr"];
    config.yellow_ycrcb_min_cb = j["yellow_ycrcb_min_cb"];
    config.yellow_ycrcb_max_cb = j["yellow_ycrcb_max_cb"];
    config.lane_mode = lane_mode_from_string(j["lane_mode"]);
    config.frame_width = j["frame_width"];
    config.frame_height = j["frame_height"];
    config.roi_top_width_coefficient = j["roi_top_width_coefficient"];
    config.roi_bottom_width_coefficient = j["roi_bottom_width_coefficient"];
    config.roi_top_y_coefficient = j["roi_top_y_coefficient"];
    config.roi_bottom_y_coefficient = j["roi_bottom_y_coefficient"];
    config.ref_hist_sigma_ratio = j["ref_hist_sigma_ratio"];
    config.ref_hist_min_weight = j["ref_hist_min_weight"];
    config.sliding_window_num_windows = j["sliding_window_num_windows"];
    config.sliding_window_margin = j["sliding_window_margin"];
    config.sliding_window_minpix = j["sliding_window_minpix"];
    config.corridor_width = j["corridor_width"];
    config.gaussian_blur_kernel_size = j["gaussian_blur_kernel_size"];
    config.canny_yellow_high_threshold = j["canny_yellow_high_threshold"];
    config.canny_yellow_low_threshold = j["canny_yellow_low_threshold"];
    config.kernel_yellow_closing_size = j["kernel_yellow_closing_size"];
    config.kernel_yellow_opening_size = j["kernel_yellow_opening_size"];
    config.center_reference_lane_one = j["center_reference_lane_one"];
    config.center_reference_lane_two = j["center_reference_lane_two"];
    config.horizontal_noise_width = j["horizontal_noise_width"];
    config.horizontal_noise_band_h = j["horizontal_noise_band_h"];
    config.horizontal_noise_extra_pad = j["horizontal_noise_extra_pad"];
    config.horizontal_band_corridor_ratio_thresh = j["horizontal_band_corridor_ratio_thresh"];
    config.vertical_noise_band_w = j["vertical_noise_band_w"];
    config.vertical_noise_min_pixels = j["vertical_noise_min_pixels"];
    config.vertical_noise_peak_ratio = j["vertical_noise_peak_ratio"];
    config.vertical_noise_extra_pad_half_width = j["vertical_noise_extra_pad_half_width"];
    config.vertical_noise_peak_use_corridor = j["vertical_noise_peak_use_corridor"];
    config.lane_change_tol_straight = j["lane_change_tol_straight"];
    config.lane_change_tol_curve = j["lane_change_tol_curve"];
    config.lane_change_tol_change = j["lane_change_tol_change"];
    config.lane_change_streak_need = j["lane_change_streak_need"];
    config.lane_change_m_split = j["lane_change_m_split"];
    config.lane_ref_transition_duration_sec = j["lane_ref_transition_duration_sec"];
    config.debug_view = j["debug_view"];
    config.change_ref_smoothly = j["change_ref_smoothly"];
    return config;
}