#pragma once
#include <opencv2/opencv.hpp>
#include "config_types.hpp"  // 파라미터 타입은 여기서 가져옴 (using 별칭 제공)

namespace lane {
namespace preprocess {

// ROI 사각형 계산 (아래 기준 퍼센트)
cv::Rect compute_roi_rect(int img_h, int img_w, int roi_bottom_pct, int roi_top_pct);

// White 마스크 (HLS+HSV)
void WMaskProcessor(const cv::Mat& input_bgr,
                    const WhiteMaskParams& p,
                    cv::Mat& out_mask,
                    cv::Mat* out_vis = nullptr);

// Yellow 마스크 (Lab+YCrCb)
void YMaskProcessor(const cv::Mat& input_bgr,
                    const YellowMaskParams& p,
                    cv::Mat& out_mask,
                    cv::Mat* out_vis = nullptr);

// White 후처리 (현재 패스스루)
void WImageProcessor(const cv::Mat& mask_in,
                     const WhiteImageParams& p,
                     cv::Mat& out);

// Yellow 후처리: Opening → Closing → Sobel X → Threshold
void YImageProcessor(const cv::Mat& mask_in,
                     const YellowImageParams& p,
                     cv::Mat& out);

} // namespace preprocess
} // namespace lane
