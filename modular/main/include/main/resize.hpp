#pragma once

#include <opencv2/opencv.hpp>

class ImageResizer {
public:
    // 고정 크기 640x360으로 리사이즈
    static cv::Mat resizeToFixed(const cv::Mat& input);
};
