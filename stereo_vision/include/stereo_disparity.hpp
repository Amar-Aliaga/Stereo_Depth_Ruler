#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include "stereo_rectifier.hpp"

class StereoDisparity {
private:
    cv::Ptr<cv::StereoSGBM> matcher;
    cv::Mat Q;

    cv::Ptr<cv::StereoMatcher> right_matcher;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
public:
    StereoDisparity(const cv::Mat &Q_matrix);
    cv::Mat computeDisparity(const cv::Mat& left, const cv::Mat& right);
    cv::Mat computeDepth(const cv::Mat& disparity);
    const cv::Ptr<cv::StereoSGBM> get_matcher() const;
};