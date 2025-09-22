#pragma once

#include "stereo_calibrator.hpp"
#include "stereo_configuration.hpp"
#include "stereo_disparity.hpp"
#include "stereo_rectifier.hpp"
#include "stereo_displayer.hpp"

class StereoPipeline {
    private:
        const StereoConfiguration &config;
        const StereoCalibrator    &calibrator;
        const StereoRectifier     &rectifier;
        const StereoDisparity     &disparity;
        const StereoDisplayer     &displayer;
    public:
        StereoPipeline(const StereoCalibrator &cal, const StereoRectifier &rect, const StereoDisparity &dis, const StereoDisplayer &display);
        StereoPipeline(const StereoCalibrator &cal);
        StereoPipeline(const StereoCalibrator &cal, const StereoRectifier &rect);
        StereoPipeline(const StereoCalibrator &cal, const StereoDisparity &dis);
};