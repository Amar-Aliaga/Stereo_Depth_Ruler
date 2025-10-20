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

        cv::Mat leftGray, rightGray;
        cv::Mat left_small, right_small;
        cv::Mat disp_left, disp_right;
        cv::Mat filtered_disp;
        cv::Mat filtered_disp_float;

        cv::Mat filtered_disp_float_masked;
        cv::Mat norm01, norm_gamma, show_disp;
        cv::Mat depthZ, z_valid, depth8u, depth_vis8u;
        cv::Mat valid_mask;

        cv::Mat depth; 
        cv::Mat depth_vis; 
    public:
        StereoDisparity(const cv::Mat &Q_matrix);

        const cv::Mat &computeDisparity(const cv::Mat& left, const cv::Mat& right);
        const cv::Mat &show_disparityMap(const cv::Mat &disparity);

        const cv::Mat &computeDepth(const cv::Mat& disparity);
        const cv::Mat &show_depthMap(const cv::Mat &disparity);

        bool process() override;

        const cv::Ptr<cv::StereoSGBM> get_matcher() const;
        const cv::Mat get_ConfidenceMap() const noexcept;
};
