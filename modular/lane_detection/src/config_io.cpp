#include "lane_detection/config_io.hpp"
#include <fstream>
#include <sstream>
#include <type_traits>
#include <algorithm>

// nlohmann-json (Ubuntu: sudo apt install nlohmann-json3-dev)
#include <nlohmann/json.hpp>
using nlohmann::json;

namespace lane {
namespace config {
namespace io {

static inline int clampi(int v, int lo, int hi) { return std::max(lo, std::min(v, hi)); }

// ---------- JSON helpers ----------

static bool read_int_pair(const json& j, const char* key, int& mn, int& mx, int def_min, int def_max) {
    mn = def_min; mx = def_max;
    if (!j.contains(key)) return false;
    const auto& v = j.at(key);
    try {
        if (v.is_array() && v.size() >= 2) {
            mn = v.at(0).get<int>();
            mx = v.at(1).get<int>();
            return true;
        }
        if (v.is_object()) {
            if (v.contains("min")) mn = v.at("min").get<int>();
            if (v.contains("max")) mx = v.at("max").get<int>();
            return true;
        }
    } catch (...) { return false; }
    return false;
}

template<typename T>
static void set_if_exists(const json& j, const char* key, T& out) {
    if (j.contains(key)) {
        try { out = j.at(key).get<T>(); } catch (...) {}
    }
}

static void parse_white_mask(const json& wm, WhiteMaskParams& p) {
    set_if_exists(wm, "roi_bottom_pct", p.roi_bottom_pct);
    set_if_exists(wm, "roi_top_pct",    p.roi_top_pct);

    if (wm.contains("lab") && wm.at("lab").is_object()) {
        const auto& lab = wm.at("lab");
        read_int_pair(lab, "l", p.lab_l_min, p.lab_l_max, p.lab_l_min, p.lab_l_max);
        read_int_pair(lab, "a", p.lab_a_min, p.lab_a_max, p.lab_a_min, p.lab_a_max);
        read_int_pair(lab, "b", p.lab_b_min, p.lab_b_max, p.lab_b_min, p.lab_b_max);
    }
    if (wm.contains("ycrcb") && wm.at("ycrcb").is_object()) {
        const auto& ycc = wm.at("ycrcb");
        read_int_pair(ycc, "y",  p.y_min,  p.y_max,  p.y_min,  p.y_max);
        read_int_pair(ycc, "cr", p.cr_min, p.cr_max, p.cr_min, p.cr_max);
        read_int_pair(ycc, "cb", p.cb_min, p.cb_max, p.cb_min, p.cb_max);
    }

    set_if_exists(wm, "visualize", p.visualize);
    p.roi_bottom_pct = clampi(p.roi_bottom_pct, 0, 100);
    p.roi_top_pct    = clampi(p.roi_top_pct,    0, 100);
}

// Yellow ← HLS + HSV
static void parse_yellow_mask(const json& ym, YellowMaskParams& p) {
    set_if_exists(ym, "roi_bottom_pct", p.roi_bottom_pct);
    set_if_exists(ym, "roi_top_pct",    p.roi_top_pct);

    if (ym.contains("hls") && ym.at("hls").is_object()) {
        const auto& hls = ym.at("hls");
        read_int_pair(hls, "h", p.hls_h_min, p.hls_h_max, p.hls_h_min, p.hls_h_max);
        read_int_pair(hls, "l", p.hls_l_min, p.hls_l_max, p.hls_l_min, p.hls_l_max);
        read_int_pair(hls, "s", p.hls_s_min, p.hls_s_max, p.hls_s_min, p.hls_s_max);
    }
    if (ym.contains("hsv") && ym.at("hsv").is_object()) {
        const auto& hsv = ym.at("hsv");
        read_int_pair(hsv, "h", p.hsv_h_min, p.hsv_h_max, p.hsv_h_min, p.hsv_h_max);
        read_int_pair(hsv, "s", p.hsv_s_min, p.hsv_s_max, p.hsv_s_min, p.hsv_s_max);
        read_int_pair(hsv, "v", p.hsv_v_min, p.hsv_v_max, p.hsv_v_min, p.hsv_v_max);
    }

    set_if_exists(ym, "visualize", p.visualize);
    p.roi_bottom_pct = clampi(p.roi_bottom_pct, 0, 100);
    p.roi_top_pct    = clampi(p.roi_top_pct,    0, 100);
}

static void parse_white_image(const json& wi, WhiteImageParams& p) {
    set_if_exists(wi, "copy_input", p.copy_input);
}

static void parse_yellow_image(const json& yi, YellowImageParams& p) {
    set_if_exists(yi, "open_kernel",  p.open_kernel);
    set_if_exists(yi, "close_kernel", p.close_kernel);
    set_if_exists(yi, "open_iters",   p.open_iters);
    set_if_exists(yi, "close_iters",  p.close_iters);

    if (yi.contains("sobel") && yi.at("sobel").is_object()) {
        const auto& sb = yi.at("sobel");
        set_if_exists(sb, "ksize",  p.sobel_ksize);
        set_if_exists(sb, "scale",  p.sobel_scale);
        set_if_exists(sb, "delta",  p.sobel_delta);
        set_if_exists(sb, "thresh", p.sobel_thresh);
    }

    // 간단한 유효성
    if (p.open_kernel  < 1) p.open_kernel  = 1;
    if (p.close_kernel < 1) p.close_kernel = 1;
    if (p.sobel_ksize  < 1) p.sobel_ksize  = 1;
}

static json dump_white_mask(const WhiteMaskParams& p) {
    return json{
        {"roi_bottom_pct", p.roi_bottom_pct},
        {"roi_top_pct",    p.roi_top_pct},
        {"lab",   { {"l", {p.lab_l_min, p.lab_l_max}},
                    {"a", {p.lab_a_min, p.lab_a_max}},
                    {"b", {p.lab_b_min, p.lab_b_max}} }},
        {"ycrcb", { {"y",  {p.y_min,  p.y_max}},
                    {"cr", {p.cr_min, p.cr_max}},
                    {"cb", {p.cb_min, p.cb_max}} }},
        {"visualize", p.visualize}
    };
}

static json dump_yellow_mask(const YellowMaskParams& p) {
    return json{
        {"roi_bottom_pct", p.roi_bottom_pct},
        {"roi_top_pct",    p.roi_top_pct},
        {"hls", { {"h", {p.hls_h_min, p.hls_h_max}},
                  {"l", {p.hls_l_min, p.hls_l_max}},
                  {"s", {p.hls_s_min, p.hls_s_max}} }},
        {"hsv", { {"h", {p.hsv_h_min, p.hsv_h_max}},
                  {"s", {p.hsv_s_min, p.hsv_s_max}},
                  {"v", {p.hsv_v_min, p.hsv_v_max}} }},
        {"visualize", p.visualize}
    };
}
static json dump_white_image(const WhiteImageParams& p) {
    return json{
        {"copy_input", p.copy_input}
    };
}

static json dump_yellow_image(const YellowImageParams& p) {
    return json{
        {"open_kernel",  p.open_kernel},
        {"close_kernel", p.close_kernel},
        {"open_iters",   p.open_iters},
        {"close_iters",  p.close_iters},
        {"sobel", {
            {"ksize",  p.sobel_ksize},
            {"scale",  p.sobel_scale},
            {"delta",  p.sobel_delta},
            {"thresh", p.sobel_thresh}
        }}
    };
}

// ---------- Public API ----------

bool LoadFromFile(const std::string& path, LaneConfig& out, std::string* error) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        if (error) *error = "Cannot open file: " + path;
        return false;
    }

    json root;
    try {
        ifs >> root;
    } catch (const std::exception& e) {
        if (error) *error = std::string("JSON parse error: ") + e.what();
        return false;
    }

    LaneConfig cfg = DefaultLaneConfig();

    try {
        if (root.contains("white_mask"))  parse_white_mask(root.at("white_mask"),  cfg.white_mask);
        if (root.contains("yellow_mask")) parse_yellow_mask(root.at("yellow_mask"), cfg.yellow_mask);
        if (root.contains("white_image")) parse_white_image(root.at("white_image"), cfg.white_image);
        if (root.contains("yellow_image"))parse_yellow_image(root.at("yellow_image"), cfg.yellow_image);

        set_if_exists(root, "version", cfg.version);
        set_if_exists(root, "note",    cfg.note);
    } catch (const std::exception& e) {
        if (error) *error = std::string("Config parse error: ") + e.what();
        return false;
    }

    out = std::move(cfg);
    return true;
}

bool SaveToFile(const std::string& path, const LaneConfig& cfg, std::string* error) {
    json root = {
        {"version", cfg.version},
        {"note",    cfg.note},
        {"white_mask",  dump_white_mask(cfg.white_mask)},
        {"yellow_mask", dump_yellow_mask(cfg.yellow_mask)},
        {"white_image", dump_white_image(cfg.white_image)},
        {"yellow_image",dump_yellow_image(cfg.yellow_image)}
    };

    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        if (error) *error = "Cannot open for write: " + path;
        return false;
    }
    try {
        ofs << root.dump(2) << std::endl;
    } catch (const std::exception& e) {
        if (error) *error = std::string("Write error: ") + e.what();
        return false;
    }
    return true;
}

LaneConfig LoadOrDefault(const std::string& path, bool* loaded_from_file, std::string* error) {
    LaneConfig cfg = DefaultLaneConfig();
    bool ok = LoadFromFile(path, cfg, error);
    if (loaded_from_file) *loaded_from_file = ok;
    return cfg;
}

} // namespace io
} // namespace config
} // namespace lane
