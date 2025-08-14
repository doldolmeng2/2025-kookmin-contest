#pragma once
#include <string>

namespace lane {
namespace config {

// =============== White / Yellow Mask Params ===============

struct WhiteMaskParams {
    int roi_bottom_pct = 10;
    int roi_top_pct    = 40;

    // ✅ White는 Lab + YCrCb
    int lab_l_min = 0,   lab_l_max = 255;
    int lab_a_min = 110, lab_a_max = 150;
    int lab_b_min = 140, lab_b_max = 200;

    int y_min  = 0,   y_max  = 255;
    int cr_min = 135, cr_max = 180;
    int cb_min = 85,  cb_max = 135;

    bool visualize = true;
};

struct YellowMaskParams {
    int roi_bottom_pct = 10;
    int roi_top_pct    = 40;

    // ✅ Yellow는 HLS + HSV
    int hls_h_min = 0,   hls_h_max = 180;
    int hls_l_min = 200, hls_l_max = 255;
    int hls_s_min = 0,   hls_s_max = 80;

    int hsv_h_min = 0,   hsv_h_max = 180;
    int hsv_s_min = 0,   hsv_s_max = 60;
    int hsv_v_min = 200, hsv_v_max = 255;

    bool visualize = true;
};

// =============== White / Yellow Image Params ===============

struct WhiteImageParams {
    bool copy_input = true; // placeholder
};

struct YellowImageParams {
    int open_kernel  = 3; // odd 권장
    int close_kernel = 5; // odd 권장
    int open_iters   = 1;
    int close_iters  = 1;

    // Sobel X
    int    sobel_ksize = 3;     // 1,3,5,7...
    double sobel_scale = 1.0;
    double sobel_delta = 0.0;
    int    sobel_thresh = 30;   // 0~255
};

// =============== Root Config ===============

struct LaneConfig {
    WhiteMaskParams  white_mask;
    YellowMaskParams yellow_mask;
    WhiteImageParams white_image;
    YellowImageParams yellow_image;

    // (옵션) 버전/메모 등
    int version = 1;
    std::string note;
};

// 기본값 팩토리
inline LaneConfig DefaultLaneConfig() { return LaneConfig{}; }

} // namespace config
} // namespace lane

// ----------------------------------------------------------
// 기존 preprocess 네임스페이스와의 호환을 위한 타입 별칭
// (preprocess.hpp에서 구조체를 제거하고 이 헤더를 include하면
//  기존 코드 변경 최소화 가능)
// ----------------------------------------------------------
namespace lane { namespace preprocess {
using WhiteMaskParams   = lane::config::WhiteMaskParams;
using YellowMaskParams  = lane::config::YellowMaskParams;
using WhiteImageParams  = lane::config::WhiteImageParams;
using YellowImageParams = lane::config::YellowImageParams;
} }
