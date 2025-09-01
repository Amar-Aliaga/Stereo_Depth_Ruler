#pragma once

#include <opencv2/opencv.hpp>
#include <string>

class StereoRectifier {
private:
    static constexpr const char *outputFile {"config/stereo.yaml"};
    cv::Mat cameraMatrixLeft, distCoeffsLeft;
    cv::Mat cameraMatrixRight, distCoeffsRight;
    cv::Mat R1, R2, P1, P2, Q;
    cv::Mat mapL1, mapL2;
    cv::Mat mapR1, mapR2;
    cv::Size imageSize;

public:
    StereoRectifier();
    bool loadCalibration(const std::string& filename);
    void rectify(const cv::Mat &left_src, const cv::Mat &right_src, cv::Mat &left_dst, cv::Mat &right_dst);
    void drawEpipolarLines(cv::Mat& rectifiedLeft, cv::Mat& rectifiedRight);
    bool run_rectification();

    const cv::Size &getImageSize() const noexcept;
    const cv::Mat  &getQ() const noexcept;
};