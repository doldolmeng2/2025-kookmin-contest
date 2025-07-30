#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

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

Config load_config(const std::string& path) {
    std::ifstream file(path);
    json j;
    file >> j;

    Config config;
    config.yellow_min_h = j["yellow_min_h"];
    config.yellow_max_h = j["yellow_max_h"];
    config.yellow_min_s = j["yellow_min_s"];
    config.yellow_max_s = j["yellow_max_s"];
    config.yellow_min_v = j["yellow_min_v"];
    config.yellow_max_v = j["yellow_max_v"];
    config.white_min_h = j["white_min_h"];
    config.white_max_h = j["white_max_h"];
    config.white_min_s = j["white_min_s"];
    config.white_max_s = j["white_max_s"];
    config.white_min_v = j["white_min_v"];
    config.white_max_v = j["white_max_v"];
    return config;
}
