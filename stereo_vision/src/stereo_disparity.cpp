#include "stereo_disparity.hpp"
#include <utility>

StereoDisparity::StereoDisparity(const cv::Mat &Q_matrix) : Q(Q_matrix) {
    matcher = cv::StereoSGBM::create(
        0, 80, 3, 8*3*3, 32*3*3, 2, 63, 15, 100, 12, cv::StereoSGBM::MODE_SGBM_3WAY
    );
    right_matcher = cv::ximgproc::createRightMatcher(matcher);
    wls_filter = cv::ximgproc::createDisparityWLSFilter(matcher);
    wls_filter->setLambda(7000.0);
    wls_filter->setSigmaColor(1.0);
}


cv::Mat StereoDisparity::computeDisparity(const cv::Mat& left, const cv::Mat& right) {
    cv::Mat leftGray, rightGray;
    cv::cvtColor(left,  leftGray,  cv::COLOR_BGR2GRAY);
    cv::cvtColor(right, rightGray, cv::COLOR_BGR2GRAY);

    cv::Mat left_small, right_small;
    cv::resize(leftGray,  left_small,  cv::Size(), 0.5, 0.5, cv::INTER_AREA);
    cv::resize(rightGray, right_small, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

    cv::Mat disp_left, disp_right;
    matcher->compute(left_small, right_small, disp_left);
    right_matcher->compute(right_small, left_small, disp_right);

    cv::Mat filtered_disp;
    wls_filter->filter(disp_left, left_small, filtered_disp, disp_right);

    cv::Mat filtered_disp_float;
    filtered_disp.convertTo(filtered_disp_float, CV_32F, 1.0 / 16.0);

    cv::Mat guided; 
    cv::ximgproc::guidedFilter(
        left_small,              // guidance image (rectified left)
        filtered_disp_float, 
        guided,    // input disparity
        5,                       // radius (try 5–9)
        1e-3                     // eps (try 1e-3 to 1e-2)
    );
    //filtered_disp_float = guided.clone();

    //cv::medianBlur(filtered_disp_float, filtered_disp_float, 3);

    const int numDisp = matcher->getNumDisparities();
    cv::Mat valid_mask = filtered_disp_float > 0;
    cv::Mat filtered_disp_float_masked = filtered_disp_float.clone();
    filtered_disp_float_masked.setTo(0, ~valid_mask);


    cv::Mat norm01;
    filtered_disp_float_masked.convertTo(norm01, CV_32F, 1.0f / std::max(1, numDisp));


    const bool apply_gamma = true;
    const double gamma = 0.6; 
    cv::Mat norm_gamma;
    if (apply_gamma) {
        cv::pow(norm01, gamma, norm_gamma);
    } else {
        norm_gamma = norm01;
    }

    cv::Mat disp_vis;
    norm_gamma.convertTo(disp_vis, CV_8U, 255.0);

    static cv::Mat prev_disp_vis;
    const float alpha = 0.45f;
    if (!prev_disp_vis.empty()) {
        cv::addWeighted(prev_disp_vis, alpha, disp_vis, 1.0f - alpha, 0.0, disp_vis);
    }
    prev_disp_vis = disp_vis.clone();

return disp_vis;
}                                                                                                                                                                                                       


cv::Mat StereoDisparity::computeDepth(const cv::Mat& disparity) {
    cv::Mat depth;
    cv::reprojectImageTo3D(disparity, depth, Q);
    return depth;
}


const cv::Ptr<cv::StereoSGBM> StereoDisparity::get_matcher() const {
    return matcher;
}