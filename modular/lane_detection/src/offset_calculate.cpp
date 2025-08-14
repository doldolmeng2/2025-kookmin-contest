#include "lane_detection/offset_calculate.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio> // std::snprintf

namespace lane {
namespace offset {

// -------------------- 내부 헬퍼 --------------------

static inline double sqr(double v){ return v*v; }

static void ensure_bgr_canvas(const cv::Mat& mask_mono, cv::Mat& viz) {
    if (viz.empty()) {
        cv::cvtColor(mask_mono, viz, cv::COLOR_GRAY2BGR);
    } else if (viz.channels() == 1) {
        cv::cvtColor(viz, viz, cv::COLOR_GRAY2BGR);
    }
}

static cv::Mat edges_from_mask(const cv::Mat& mask_mono, const DetectParams& p) {
    if (p.canny_low > 0 && p.canny_high > 0) {
        cv::Mat edges;
        cv::Canny(mask_mono, edges, p.canny_low, p.canny_high, p.canny_aperture);
        return edges;
    }
    // 바이너리 마스크를 그대로 사용
    return mask_mono;
}

static double seg_length(const cv::Vec4i& l) {
    return std::hypot(static_cast<double>(l[2]-l[0]),
                      static_cast<double>(l[3]-l[1]));
}

#if 0
static bool point_in_norm_rect(const cv::Point& pt, const cv::Size& sz,
                               double xmin, double xmax, double ymin, double ymax) {
    double x = static_cast<double>(pt.x) / std::max(1, sz.width);
    double y = static_cast<double>(pt.y) / std::max(1, sz.height);
    return (x >= xmin && x <= xmax && y >= ymin && y <= ymax);
}
#endif

// seg → 무한 직선으로 확장(이미지 경계와의 교점 2개 구하기)
static bool extend_to_image(const cv::Point2f& a, const cv::Point2f& b,
                            const cv::Size& sz, cv::Point2f& q1, cv::Point2f& q2) {
    cv::Vec4f line;
    std::vector<cv::Point2f> pts = {a, b};
    cv::fitLine(pts, line, cv::DIST_L2, 0, 1e-2, 1e-2);
    cv::Point2f p0(line[2], line[3]);
    cv::Point2f v (line[0], line[1]);
    if (std::abs(v.x) < 1e-6 && std::abs(v.y) < 1e-6) return false;

    // 이미지 경계와의 교점 후보 계산
    std::vector<cv::Point2f> cand;
    auto add_if_inside = [&](const cv::Point2f& pt){
        if (pt.x >= 0 && pt.x <= sz.width-1 && pt.y >= 0 && pt.y <= sz.height-1) {
            cand.push_back(pt);
        }
    };

    // x=0, x=W-1
    if (std::abs(v.x) > 1e-6) {
        double t0 = (0 - p0.x) / v.x;
        add_if_inside(cv::Point2f(0, p0.y + t0 * v.y));
        double t1 = (sz.width-1 - p0.x) / v.x;
        add_if_inside(cv::Point2f(sz.width-1, p0.y + t1 * v.y));
    }
    // y=0, y=H-1
    if (std::abs(v.y) > 1e-6) {
        double t2 = (0 - p0.y) / v.y;
        add_if_inside(cv::Point2f(p0.x + t2 * v.x, 0));
        double t3 = (sz.height-1 - p0.y) / v.y;
        add_if_inside(cv::Point2f(p0.x + t3 * v.x, sz.height-1));
    }

    // 서로 다른 두 점 선택
    if (cand.size() < 2) return false;
    // 가장 멀리 떨어진 두 점 선택
    double bestd = -1; int bi=0, bj=1;
    for (int i=0;i<(int)cand.size();++i){
        for (int j=i+1;j<(int)cand.size();++j){
            double d = sqr(cand[i].x - cand[j].x) + sqr(cand[i].y - cand[j].y);
            if (d > bestd){ bestd = d; bi=i; bj=j; }
        }
    }
    q1 = cand[bi]; q2 = cand[bj];
    return true;
}

// 두 무한직선의 교점 (각각 두 점 표현)
static bool line_intersection(const cv::Point2f& a1, const cv::Point2f& a2,
                              const cv::Point2f& b1, const cv::Point2f& b2,
                              cv::Point2f& out) {
    cv::Point2f r = a2 - a1;
    cv::Point2f s = b2 - b1;
    float rxs = r.x*s.y - r.y*s.x;
    if (std::abs(rxs) < 1e-6f) return false; // 평행
    cv::Point2f qp = b1 - a1;
    float t = (qp.x * s.y - qp.y * s.x) / rxs;
    out = a1 + t * r;
    return true;
}

static void draw_line(cv::Mat& viz, const cv::Point2f& p1, const cv::Point2f& p2, const cv::Scalar& color, int thickness=2) {
    cv::line(viz, p1, p2, color, thickness, cv::LINE_AA);
}

static void draw_cross(cv::Mat& viz, const cv::Point2f& c, int size, const cv::Scalar& color, int thickness=2) {
    cv::line(viz, {int(c.x)-size, int(c.y)}, {int(c.x)+size, int(c.y)}, color, thickness, cv::LINE_AA);
    cv::line(viz, {int(c.x), int(c.y)-size}, {int(c.x), int(c.y)+size}, color, thickness, cv::LINE_AA);
}

// --- 최근 유효 라인/오프셋 히스토리 ---
struct History {
    LineFit   last_w;
    LineFit   last_y;
    OffsetViz last_offset;
    int miss_w = 0;
    int miss_y = 0;
};
static History g_hist;

// -------------------- 공개 함수 --------------------

LineFit WLineCalculate(const cv::Mat& mask_mono,
                       cv::Mat& viz,
                       const DetectParams& p)
{
    CV_Assert(!mask_mono.empty() && mask_mono.channels() == 1);
    ensure_bgr_canvas(mask_mono, viz);
    const cv::Size sz = mask_mono.size();

    cv::Mat edges = edges_from_mask(mask_mono, p);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, p.rho, p.theta, p.hough_thresh,
                    p.min_line_length, p.max_line_gap);

    // 시작점(혹은 끝점) 중 하나가 "우하단 시작 영역" 안에 있고 길이가 긴 선분 선택
    cv::Rect start_roi(
        int(p.start_x_min_frac * sz.width),
        int(p.start_y_min_frac * sz.height),
        int((p.start_x_max_frac - p.start_x_min_frac) * sz.width),
        int((p.start_y_max_frac - p.start_y_min_frac) * sz.height)
    );

    LineFit best;
    double best_len = -1.0;
    for (const auto& l : lines) {
        cv::Point s(l[0], l[1]);
        cv::Point e(l[2], l[3]);
        double L = seg_length(l);
        if (L < p.min_line_length) continue;

        bool start_ok = start_roi.contains(s) || start_roi.contains(e);
        if (!start_ok) continue;

        // 연장
        cv::Point2f q1, q2;
        if (!extend_to_image(s, e, sz, q1, q2)) continue;

        if (L > best_len) {
            best_len = L;
            best.valid = true;
            best.p1 = q1; best.p2 = q2;
            best.seg_length = static_cast<float>(L);
        }
    }

    // 시각화 & 히스토리/폴백
    if (best.valid) {
        cv::rectangle(viz, start_roi, cv::Scalar(80, 255, 80), 1, cv::LINE_AA);
        draw_line(viz, best.p1, best.p2, cv::Scalar(255,255,255), 3); // 흰색 라인
        g_hist.last_w = best;
        g_hist.miss_w = 0;
    } else {
        if (p.hold_frames > 0 && g_hist.last_w.valid && g_hist.miss_w < p.hold_frames) {
            ++g_hist.miss_w;
            best = g_hist.last_w;
            cv::rectangle(viz, start_roi, cv::Scalar(80, 200, 80), 1, cv::LINE_AA);
            draw_line(viz, best.p1, best.p2, cv::Scalar(200,200,200), 2);
            cv::putText(viz, "W HOLD", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(200,200,200), 2, cv::LINE_AA);
        } else {
            g_hist.miss_w = 0;
        }
    }

    return best;
}

LineFit YLineCalculate(const cv::Mat& mask_mono,
                       cv::Mat& viz,
                       const DetectParams& p,
                       const LineFit* white_line,
                       OffsetViz* out_offset)
{
    CV_Assert(!mask_mono.empty() && mask_mono.channels() == 1);
    ensure_bgr_canvas(mask_mono, viz);
    const cv::Size sz = mask_mono.size();
    const int H = sz.height;

    // 1) 점선 메우기 + 약간 굵게
    cv::Mat work = mask_mono.clone();
    int k = (p.yellow_bridge_kernel > 0) ? p.yellow_bridge_kernel : 1;
    if (k % 2 == 0) k++;
    cv::Mat kv = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, k));
    cv::morphologyEx(work, work, cv::MORPH_CLOSE, kv, cv::Point(-1,-1), 1);

    cv::Mat k3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::dilate(work, work, k3, cv::Point(-1,-1), 1);

    // 2) 모든 흰 픽셀
    std::vector<cv::Point> pts;
    cv::findNonZero(work, pts);

    // ---- RANSAC 파라미터(고정 상수) ----
    const int   kMinPoints      = 20;
    const float kInlierDistPx   = 2.5f;
    const int   kRansacIters    = 200;
    const float kMinVisualLenPx = std::max<float>(float(p.min_line_length), 5.f);

    LineFit best;

    if ((int)pts.size() >= kMinPoints) {
        auto dist_pt_line = [](const cv::Point2f& P, float a, float b, float c){
            return std::abs(a*P.x + b*P.y + c) / std::sqrt(a*a + b*b);
        };

        cv::RNG rng(12345);
        int best_inliers = -1;
        float best_a=0, best_b=0, best_c=0;

        // 3) RANSAC
        for (int it=0; it<kRansacIters; ++it) {
            int i = rng.uniform(0, (int)pts.size());
            int j = rng.uniform(0, (int)pts.size());
            if (i == j) continue;
            cv::Point2f p1 = pts[i], p2 = pts[j];
            if (cv::norm(p1 - p2) < 2.0) continue;

            float a = p2.y - p1.y;
            float b = p1.x - p2.x;
            float c = -(a*p1.x + b*p1.y);

            int inliers = 0;
            for (const auto& q : pts) {
                if (dist_pt_line(q, a,b,c) <= kInlierDistPx) ++inliers;
            }
            if (inliers > best_inliers) {
                best_inliers = inliers; best_a=a; best_b=b; best_c=c;
            }
        }

        // 4) 인라이어로 최종 피팅 + 각도/길이 체크
        if (best_inliers > 0) {
            std::vector<cv::Point2f> inliers;
            inliers.reserve(best_inliers);
            for (const auto& q : pts) {
                float d = std::abs(best_a*q.x + best_b*q.y + best_c) / std::sqrt(best_a*best_a + best_b*best_b);
                if (d <= kInlierDistPx) inliers.emplace_back((float)q.x, (float)q.y);
            }

            if ((int)inliers.size() >= kMinPoints/2) {
                cv::Vec4f line;
                cv::fitLine(inliers, line, cv::DIST_L2, 0, 1e-2, 1e-2);
                cv::Point2f p0(line[2], line[3]), v(line[0], line[1]);

                // (A) 각도 제한 (절대값)
                double ang = std::abs(std::atan2(double(v.y), double(v.x))) * 180.0 / CV_PI;
                if (ang >= p.y_angle_min_deg && ang <= p.y_angle_max_deg) {
                    // (B) 화면 경계로 연장 + 충분한 시각화 길이
                    cv::Point2f q1, q2;
                    if (extend_to_image(p0 - 1000*v, p0 + 1000*v, sz, q1, q2)) {
                        if (cv::norm(q1 - q2) >= kMinVisualLenPx) {
                            best.valid = true;
                            best.p1 = q1; best.p2 = q2;
                            best.seg_length = (float)cv::norm(q1 - q2);
                        }
                    }
                }
            }
        }
    }

    // 5) 노란 선: 검출/홀드 로직
    if (best.valid) {
        draw_line(viz, best.p1, best.p2, cv::Scalar(0,255,255), 3);
        g_hist.last_y = best;
        g_hist.miss_y = 0;
    } else {
        if (p.hold_frames > 0 && g_hist.last_y.valid && g_hist.miss_y < p.hold_frames) {
            ++g_hist.miss_y;
            best = g_hist.last_y;
            draw_line(viz, best.p1, best.p2, cv::Scalar(0,200,200), 2);
            cv::putText(viz, "Y HOLD", {10, 55}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0,200,200), 2, cv::LINE_AA);
        } else {
            g_hist.miss_y = 0;
        }
    }

    // 6) 오프셋 계산 (교점/검증/홀드)
    if (out_offset) *out_offset = {};
    bool new_offset_ok = false;
    OffsetViz new_off;

    if (white_line && white_line->valid && best.valid) {
        cv::Point2f inter;
        if (line_intersection(white_line->p1, white_line->p2, best.p1, best.p2, inter)) {
            double cx = (sz.width - 1) * 0.5;
            double offset_px   = inter.x - cx;
            double offset_norm = offset_px / (sz.width * 0.5);

            // (C) 교점 검증 규칙
            bool y_in_bottom40 = (inter.y > 0.6 * H);        // 하단 40%에 교점 뜨면 비정상
            bool too_large     = (std::abs(offset_px) >= 500); // offset 크기 제한

            if (!y_in_bottom40 && !too_large) {
                new_offset_ok = true;
                new_off.valid = true;
                new_off.intersection = inter;
                new_off.offset_px = offset_px;
                new_off.offset_norm = offset_norm;

                // 시각화
                draw_cross(viz, inter, 6, cv::Scalar(255,0,255), 2);
                char buf[128];
                std::snprintf(buf, sizeof(buf), "offset: %+0.1fpx (norm %+0.3f)",
                              offset_px, offset_norm);
                int base=0; cv::Size tsize = cv::getTextSize(buf, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &base);
                cv::rectangle(viz, cv::Rect(10,10, tsize.width+12, tsize.height+12),
                              cv::Scalar(0,0,0), cv::FILLED);
                cv::putText(viz, buf, {16, 10+tsize.height+2}, cv::FONT_HERSHEY_SIMPLEX,
                            0.6, cv::Scalar(255,255,255), 2, cv::LINE_AA);
            }
        }
    }

    // (D) 새 오프셋이 정상이면 업데이트, 아니면 HOLD
    if (new_offset_ok) {
        g_hist.last_offset = new_off;
        if (out_offset) *out_offset = new_off;
    } else {
        if (g_hist.last_offset.valid) {
            // 이전 offset 유지 (표시도 유지)
            if (out_offset) *out_offset = g_hist.last_offset;
            draw_cross(viz, g_hist.last_offset.intersection, 6, cv::Scalar(160,0,160), 2);
            cv::putText(viz, "OFFSET HOLD", {10, 85}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(160,0,160), 2, cv::LINE_AA);
        }
    }

    // 흰 선 시각화(항상)
    if (white_line && white_line->valid) {
        draw_line(viz, white_line->p1, white_line->p2, cv::Scalar(255,255,255), 3);
    }

    return best;
}

} // namespace offset
} // namespace lane
