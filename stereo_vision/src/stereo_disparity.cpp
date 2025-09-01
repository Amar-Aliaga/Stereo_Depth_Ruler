#include "stereo_disparity.hpp"

StereoDisparity::StereoDisparity(const cv::Mat &Q_matrix) : Q(Q_matrix) {
    matcher = cv::StereoSGBM::create(
        0, 128, 9, 8*9*9, 32*9*9, 2, 63, 3, 100, 12, cv::StereoSGBM::MODE_SGBM_3WAY
    );
    right_matcher = cv::ximgproc::createRightMatcher(matcher);
    wls_filter = cv::ximgproc::createDisparityWLSFilter(matcher);
    wls_filter->setLambda(8000.0);
    wls_filter->setSigmaColor(1.5);
    //wls_filter->setLRCrossCheckingValue(24);
}


cv::Mat StereoDisparity::computeDisparity(const cv::Mat& left, const cv::Mat& right) {
    cv::Mat leftGray, rightGray;

    cv::cvtColor(left,  leftGray,  cv::COLOR_BGR2GRAY);
    cv::cvtColor(right, rightGray, cv::COLOR_BGR2GRAY);

    cv::Mat left_small, right_small;
    cv::resize(leftGray, left_small, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR_EXACT);
    cv::resize(rightGray, right_small, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR_EXACT);

    cv::Mat disp_left, disp_right;
    matcher->compute(left_small, right_small, disp_left);
    right_matcher->compute(right_small, left_small, disp_right);

    cv::Mat filtered_disp;
    wls_filter->filter(disp_left, left_small, filtered_disp, disp_right);

    cv::Mat filtered_disp_float;
    filtered_disp.convertTo(filtered_disp_float, CV_32F, 1.0 / 16.0);

    cv::Mat disp_norm;
    cv::normalize(filtered_disp_float, disp_norm, 0, 255, cv::NORM_MINMAX, CV_8U);

    cv::Mat valid_mask = filtered_disp_float > 0;

    cv::Mat filter_disp_norm = cv::Mat::zeros(filtered_disp_float.size(), CV_8U);
    double minVal, maxVal;
    cv::minMaxLoc(filtered_disp_float, &minVal, &maxVal, nullptr, nullptr, valid_mask);
    
    if (maxVal > minVal && maxVal > 0) {
        cv::Mat temp;
        filtered_disp_float.convertTo(temp, CV_8U, 255.0 / maxVal, 0);
        temp.copyTo(filter_disp_norm, valid_mask);  
    }

    return filtered_disp_float;
}


cv::Mat StereoDisparity::computeDepth(const cv::Mat& disparity) {
    cv::Mat depth;
    cv::reprojectImageTo3D(disparity, depth, Q);
    return depth;
}


const cv::Ptr<cv::StereoSGBM> StereoDisparity::get_matcher() const {
    return matcher;
}