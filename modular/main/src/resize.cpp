#include "main/resize.hpp"

cv::Mat ImageResizer::resizeToFixed(const cv::Mat& input) {
    cv::Mat output;
    cv::resize(input, output, cv::Size(640, 360)); // 고정 크기
    return output;
}
