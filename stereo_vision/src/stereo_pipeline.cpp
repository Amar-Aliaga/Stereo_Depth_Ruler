#include "stereo_pipeline.hpp"
#include "stereo_calibrator.hpp"
#include "stereo_configuration.hpp"
#include "stereo_disparity.hpp"
#include "stereo_rectifier.hpp"
#include "stereo_displayer.hpp"

StereoPipeline::StereoPipeline(const StereoCalibrator &cal, const StereoRectifier &rect, const cv::Mat& Q_matrix, const StereoDisplayer &display) :
    calibrate(cal.calibrate(outputFile)), rectifier(cal.getConfig()), disparity(Q_matrix), displayer(display) {}