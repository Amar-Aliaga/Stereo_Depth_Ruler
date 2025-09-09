#include "stereo_disparity.hpp"
#include <utility>

StereoDisparity::StereoDisparity(const cv::Mat &Q_matrix) : Q(Q_matrix) {
    matcher = cv::StereoSGBM::create(
        //0, 80, 3, 8*3*3, 32*3*3, 2, 63, 15, 100, 12, cv::StereoSGBM::MODE_SGBM_3WAY
        0, 90, 5, 8*5*5, 32*5*5, 2, 45, 11, 75, 12, cv::StereoSGBM::MODE_SGBM_3WAY
        
    );
    right_matcher = cv::ximgproc::createRightMatcher(matcher);
    wls_filter = cv::ximgproc::createDisparityWLSFilter(matcher);
    wls_filter->setLambda(12000.0);
    wls_filter->setSigmaColor(1.7);
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

    cv::Mat conf_map = wls_filter->getConfidenceMap();

    return filtered_disp_float;
}     


cv::Mat StereoDisparity::show_disparityMap(const cv::Mat &disparity) {
    const int numDisp = matcher->getNumDisparities();
    cv::Mat valid_mask = disparity > 0;
    cv::Mat filtered_disp_float_masked = disparity.clone();
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

    cv::Mat show_disp;
    norm_gamma.convertTo(show_disp, CV_8U, 255.0);

    cv::Mat working_disp = disparity.clone();
    
    if (!prev_vis.empty() && prev_vis.size() == show_disp.size()) {
        const float alpha = 0.41f; 
        cv::addWeighted(prev_vis, alpha, show_disp, 1 - alpha, 0, show_disp);
    }
    prev_vis = show_disp.clone();
    
    return prev_vis;
}


cv::Mat StereoDisparity::computeDepth(const cv::Mat& disparity) {
    cv::Mat depth;
    cv::reprojectImageTo3D(disparity, depth, Q);
    return depth;
}


cv::Mat StereoDisparity::show_depthMap(const cv::Mat &disparity) {
    cv::Mat depthZ;
    if (disparity.channels() == 3) {
        cv::extractChannel(disparity, depthZ, 2);  
    } else {
        depthZ = disparity;
    }
    
    static double zmin_smooth = 1000.0, zmax_smooth = 2000.0;

    cv::Mat z_valid = (depthZ > 0) & (depthZ < 10000) & (depthZ == depthZ);

    double zmin_raw = 0, zmax_raw = 0;
    cv::minMaxLoc(depthZ, &zmin_raw, &zmax_raw, nullptr, nullptr, z_valid);

    if (!(zmax_raw > zmin_raw)) {
        zmin_raw = 1000.0;
        zmax_raw = 2000.0;
    }

    double alpha = 0.1;
    zmin_smooth = (1.0 - alpha) * zmin_smooth + alpha * zmin_raw;
    zmax_smooth = (1.0 - alpha) * zmax_smooth + alpha * zmax_raw;

    zmin_smooth = std::max(0.0, std::min(zmin_smooth, 10000.0));
    zmax_smooth = std::max(zmin_smooth + 1.0, std::min(zmax_smooth, 10000.0));

    cv::Mat depth8u;
    depthZ.convertTo(depth8u, CV_8U, 255.0 / (zmax_smooth - zmin_smooth), -255.0 * zmin_smooth / (zmax_smooth - zmin_smooth));

    cv::Mat depth_vis8u;
    cv::applyColorMap(depth8u, depth_vis8u, cv::COLORMAP_TURBO);

    if (!prev_depth_vis.empty() && prev_depth_vis.size() == depth_vis8u.size()) {
        const float vis_alpha = 0.41f;
        cv::addWeighted(prev_depth_vis, vis_alpha, depth_vis8u, 1.0f - vis_alpha, 0, depth_vis8u);
    }
    
    prev_depth_vis = depth_vis8u.clone();

    return depth_vis8u;
}


const cv::Ptr<cv::StereoSGBM> StereoDisparity::get_matcher() const {
    return matcher;
}