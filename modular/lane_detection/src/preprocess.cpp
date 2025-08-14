#include "lane_detection/preprocess.hpp"
#include <algorithm>

namespace lane {
namespace preprocess {

static inline int clampi(int v, int lo, int hi) { return std::max(lo, std::min(v, hi)); }
static inline int odd_or_next(int k) { return (k % 2 == 1) ? k : (k + 1); }

cv::Rect compute_roi_rect(int img_h, int img_w, int roi_bottom_pct, int roi_top_pct) {
    roi_bottom_pct = clampi(roi_bottom_pct, 0, 100);
    roi_top_pct    = clampi(roi_top_pct,    0, 100);

    // 보정: top은 bottom보다 커야 '더 위쪽' 범위가 됨
    if (roi_top_pct < roi_bottom_pct) std::swap(roi_top_pct, roi_bottom_pct);

    // 아래 기준 백분율 → y 좌표
    // 예: h=720, bottom=10, top=40 -> y_start = 720 - 0.40*720, y_end = 720 - 0.10*720
    int y_start = img_h - static_cast<int>(img_h * (roi_top_pct / 100.0));
    int y_end   = img_h - static_cast<int>(img_h * (roi_bottom_pct / 100.0));

    y_start = clampi(y_start, 0, img_h);
    y_end   = clampi(y_end,   0, img_h);
    int roi_h = std::max(0, y_end - y_start);

    return cv::Rect(0, y_start, img_w, roi_h);
}

// 공통: ROI 마스크를 전체 크기로 복원/삽입
static void paste_mask_to_full(const cv::Size& full_size, const cv::Rect& roi, const cv::Mat& roi_mask, cv::Mat& out_full) {
    out_full = cv::Mat::zeros(full_size, CV_8UC1);
    if (roi.width <= 0 || roi.height <= 0) return;
    roi_mask.copyTo(out_full(roi));
}

// 공통: 시각화(ROI 테두리 + 마스크 컬러 오버레이)
static void build_overlay(const cv::Mat& input_bgr, const cv::Rect& roi, const cv::Mat& full_mask, cv::Mat& vis_out) {
    vis_out = input_bgr.clone();
    // mask를 색상으로 오버레이
    cv::Mat colored;
    cv::cvtColor(full_mask, colored, cv::COLOR_GRAY2BGR);
    // 마스크 픽셀에만 색을 주기 위해 컬러 스케일
    cv::Mat mask_colored = cv::Mat::zeros(colored.size(), colored.type());
    // 초록색 계열 오버레이
    mask_colored.setTo(cv::Scalar(0, 255, 0), full_mask);
    cv::addWeighted(vis_out, 1.0, mask_colored, 0.3, 0.0, vis_out);

    // ROI 표시
    if (roi.width > 0 && roi.height > 0) {
        cv::rectangle(vis_out, roi, cv::Scalar(0, 0, 255), 2);
    }
}

// ✅ White: Lab -> apply to ROI -> YCrCb (sequential)
// 최종 out_mask = YCrCb threshold 결과 (Lab은 전처리 단계로만 사용)
// out_vis가 있으면 [Lab mask | YCrCb(final)] 마스크 2분할로 시각화
void WMaskProcessor(const cv::Mat& input_bgr,
                    const WhiteMaskParams& p,
                    cv::Mat& out_mask,
                    cv::Mat* out_vis) {
    CV_Assert(!input_bgr.empty() && input_bgr.channels() == 3);
    const int H = input_bgr.rows, W = input_bgr.cols;

    // ROI
    cv::Rect roi = compute_roi_rect(H, W, p.roi_bottom_pct, p.roi_top_pct);
    const cv::Mat roi_bgr = (roi.height > 0) ? input_bgr(roi) : input_bgr;

    // 1) Lab threshold -> lab_mask
    cv::Mat lab, lab_mask;
    cv::cvtColor(roi_bgr, lab, cv::COLOR_BGR2Lab);
    cv::inRange(lab,
                cv::Scalar(p.lab_l_min, p.lab_a_min, p.lab_b_min),
                cv::Scalar(p.lab_l_max, p.lab_a_max, p.lab_b_max),
                lab_mask);

    // 2) Lab 마스크를 ROI 원본에 적용 -> step1_bgr
    cv::Mat step1_bgr;
    cv::bitwise_and(roi_bgr, roi_bgr, step1_bgr, lab_mask);

    // 3) step1_bgr에 YCrCb threshold -> ycc_mask (== 최종)
    cv::Mat ycrcb, ycc_mask;
    cv::cvtColor(step1_bgr, ycrcb, cv::COLOR_BGR2YCrCb);
    cv::inRange(ycrcb,
                cv::Scalar(p.y_min, p.cr_min, p.cb_min),
                cv::Scalar(p.y_max, p.cr_max, p.cb_max),
                ycc_mask);

    // 4) 최종 마스크는 ycc_mask 그대로
    paste_mask_to_full(input_bgr.size(), roi, ycc_mask, out_mask);

    // ---- mask-only visualization (Lab | YCrCb(final)) ----
    if (out_vis && p.visualize) {
        cv::Mat full_lab_mask, full_ycc_mask;
        paste_mask_to_full(input_bgr.size(), roi, lab_mask, full_lab_mask);
        paste_mask_to_full(input_bgr.size(), roi, ycc_mask, full_ycc_mask);

        auto to_bgr_with_label = [](const cv::Mat& mono, const std::string& label) {
            cv::Mat bgr; cv::cvtColor(mono, bgr, cv::COLOR_GRAY2BGR);
            cv::putText(bgr, label, {12, 30}, cv::FONT_HERSHEY_SIMPLEX,
                        0.8, cv::Scalar(255,255,255), 2, cv::LINE_AA);
            return bgr;
        };

        cv::Mat vis_lab   = to_bgr_with_label(full_lab_mask,   "Lab");
        cv::Mat vis_ycc   = to_bgr_with_label(full_ycc_mask,   "YCrCb (final)");

        cv::hconcat(std::vector<cv::Mat>{vis_lab, vis_ycc}, *out_vis);
    }
}



// ✅ Yellow: HLS -> apply mask -> HSV (sequential)
void YMaskProcessor(const cv::Mat& input_bgr,
                    const YellowMaskParams& p,
                    cv::Mat& out_mask,
                    cv::Mat* out_vis) {
    CV_Assert(!input_bgr.empty() && input_bgr.channels() == 3);
    const int H = input_bgr.rows, W = input_bgr.cols;

    // ROI 선택
    cv::Rect roi = compute_roi_rect(H, W, p.roi_bottom_pct, p.roi_top_pct);
    const cv::Mat roi_bgr = (roi.height > 0) ? input_bgr(roi) : input_bgr;

    // 1) HLS threshold
    cv::Mat hls, hls_mask;
    cv::cvtColor(roi_bgr, hls, cv::COLOR_BGR2HLS);
    cv::inRange(hls,
                cv::Scalar(p.hls_h_min, p.hls_l_min, p.hls_s_min),
                cv::Scalar(p.hls_h_max, p.hls_l_max, p.hls_s_max),
                hls_mask);

    // 2) HLS 마스크를 원본 ROI에 적용
    cv::Mat step1_bgr;
    cv::bitwise_and(roi_bgr, roi_bgr, step1_bgr, hls_mask);

    // 3) (마스크가 적용된 영상에) HSV threshold
    cv::Mat hsv, hsv_mask;
    cv::cvtColor(step1_bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
                cv::Scalar(p.hsv_h_min, p.hsv_s_min, p.hsv_v_min),
                cv::Scalar(p.hsv_h_max, p.hsv_s_max, p.hsv_v_max),
                hsv_mask);

    // 4) 최종 마스크 = 직렬 효과 (사실상 AND)
    cv::Mat roi_mask;
    cv::bitwise_and(hls_mask, hsv_mask, roi_mask);

    // 전체 크기로 복원 + 시각화
    paste_mask_to_full(input_bgr.size(), roi, roi_mask, out_mask);
    if (out_vis && p.visualize) {
        build_overlay(input_bgr, roi, out_mask, *out_vis);
    }
}


// -------------------------------
// White Image (placeholder)
// -------------------------------
void WImageProcessor(const cv::Mat& mask_in,
                     const WhiteImageParams& p,
                     cv::Mat& out) {
    CV_Assert(!mask_in.empty());
    if (p.copy_input) {
        mask_in.copyTo(out);
    } else {
        out = mask_in.clone();
    }
}

// -------------------------------
// Yellow Image: Opening → Closing → SobelX → Threshold
// -------------------------------
void YImageProcessor(const cv::Mat& mask_in,
                     const YellowImageParams& p,
                     cv::Mat& out) {
    CV_Assert(!mask_in.empty());
    CV_Assert(mask_in.type() == CV_8UC1 || mask_in.type() == CV_8UC3);

    cv::Mat gray;
    if (mask_in.channels() == 3) {
        cv::cvtColor(mask_in, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = mask_in;
    }

    // Morphology (open → close)
    int k_open  = std::max(1, odd_or_next(p.open_kernel));
    int k_close = std::max(1, odd_or_next(p.close_kernel));
    cv::Mat kernel_open  = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k_open,  k_open));
    cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k_close, k_close));

    cv::Mat morphed;
    cv::morphologyEx(gray, morphed, cv::MORPH_OPEN,  kernel_open,  cv::Point(-1,-1), std::max(0, p.open_iters));
    cv::morphologyEx(morphed, morphed, cv::MORPH_CLOSE, kernel_close, cv::Point(-1,-1), std::max(0, p.close_iters));

    // Sobel X
    int ksize = std::max(1, odd_or_next(p.sobel_ksize));
    cv::Mat grad_x16, grad_x8;
    // cv::Sobel(morphed, grad_x16, CV_16S, 1, 0, ksize, p.sobel_scale, p.sobel_delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x16, grad_x8);
    
    // Threshold
    if (p.sobel_thresh > 0) {
        cv::threshold(grad_x8, out, p.sobel_thresh, 255, cv::THRESH_BINARY);
    } else {
        out = grad_x8; // 그대로 출력
    }
}

} // namespace preprocess
} // namespac
