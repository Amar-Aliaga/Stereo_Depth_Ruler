#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

struct StereoConfiguration {
    cv::Mat cameraMatrixLeft, distCoeffsLeft;
    cv::Mat cameraMatrixRight, distCoeffsRight;
    cv::Mat R, T, E, F;
    cv::Mat R1, R2, P1, P2, Q;
    cv::Size imageSize;

    bool loadFromFile(const std::string &filename);
    bool saveToFile(const std::string &filename);
    bool isValid() const;
};