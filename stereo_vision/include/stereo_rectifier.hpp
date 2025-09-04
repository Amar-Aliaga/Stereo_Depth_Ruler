#pragma once

#include "stereo_configuration.hpp"
#include <opencv2/opencv.hpp>
#include <string>

class StereoRectifier {
private:
    StereoConfiguration config;

    cv::Mat mapL1, mapL2;
    cv::Mat mapR1, mapR2;

public:
    explicit StereoRectifier(const StereoConfiguration &config);
   // bool loadCalibration(const std::string& filename);
    void rectify(const cv::Mat &left_src, const cv::Mat &right_src, cv::Mat &left_dst, cv::Mat &right_dst);
    void drawEpipolarLines(cv::Mat& rectifiedLeft, cv::Mat& rectifiedRight);
    bool run_rectification();

    const cv::Size &getImageSize() const noexcept;
    const cv::Mat  &getQ() const noexcept;
};