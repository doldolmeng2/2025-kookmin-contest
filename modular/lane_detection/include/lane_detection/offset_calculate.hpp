#pragma once
#include <opencv2/opencv.hpp>

namespace lane {
namespace offset {

struct LineFit {
    bool        valid = false;
    cv::Point2f p1, p2;      // 화면 경계까지 연장된 두 점
    float       seg_length = 0.f; // 원본(허프) 선분 길이 기반 스코어
};

struct OffsetViz {
    bool        valid = false;
    cv::Point2f intersection{};
    double      offset_px   = 0.0;
    double      offset_norm = 0.0;
};

struct DetectParams {
    // --- HoughLinesP ---
    double rho = 1.0;
    double theta = CV_PI / 180.0;
    int    hough_thresh = 2;
    double min_line_length = 1.0;
    double max_line_gap    = 400.0;

    // --- (옵션) Canny ---
    int canny_low = 0;
    int canny_high = 0;
    int canny_aperture = 3;

    // --- White 시작 ROI (우하단) - 비율(0~1) ---
    double start_x_min_frac = 0.65;
    double start_x_max_frac = 0.95;
    double start_y_min_frac = 0.65;
    double start_y_max_frac = 0.95;

    // --- Yellow(점선) 특화 ---
    int    yellow_bridge_kernel = 1; // 세로 close 커널(홀수). 0=off
    bool   y_use_cluster_fit = true;
    double y_angle_min_deg = 15.0;
    double y_angle_max_deg = 88.0;
    bool   y_force_left_half = false;
    int    y_min_segments_for_fit = 3;
    double y_fit_outlier_px = 6.0;

    // --- 검출 실패 시 최근값 유지 ---
    int hold_frames = 8; // 0이면 유지 기능 off
};

// 흰/노란 라인 계산
LineFit WLineCalculate(const cv::Mat& mask_mono, cv::Mat& viz, const DetectParams& p);
LineFit YLineCalculate(const cv::Mat& mask_mono, cv::Mat& viz, const DetectParams& p,
                       const LineFit* white_line, OffsetViz* out_offset);

} // namespace offset
} // namespace lane
