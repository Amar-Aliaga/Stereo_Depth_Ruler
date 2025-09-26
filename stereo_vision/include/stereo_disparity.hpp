#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/ximgproc/edge_filter.hpp>

#include "stereo_rectifier.hpp"
#include "stereo_pipeline.hpp"

class StereoDisparity : public StereoPipeline{
private:
    cv::Ptr<cv::StereoSGBM> matcher;
    cv::Mat Q, prev_vis, prev_depth_vis, confidence_map;

    cv::Ptr<cv::StereoMatcher> right_matcher;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
public:
    StereoDisparity(const cv::Mat &Q_matrix);

    cv::Mat computeDisparity(const cv::Mat& left, const cv::Mat& right);
    cv::Mat show_disparityMap(const cv::Mat &disparity);

    cv::Mat computeDepth(const cv::Mat& disparity);
    cv::Mat show_depthMap(const cv::Mat &disparity);

    bool process() override;

    const cv::Ptr<cv::StereoSGBM> get_matcher() const;
    const cv::Mat get_ConfidenceMap() const;
};
