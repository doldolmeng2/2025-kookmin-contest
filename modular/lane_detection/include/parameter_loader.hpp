
#pragma once
#include <string>

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
};

Config load_config(const std::string& path);
