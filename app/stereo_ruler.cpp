// #include <opencv2/opencv.hpp>
// #include <opencv2/ximgproc.hpp>
#include "stereo_calibrator.hpp"
#include "stereo_rectifier.hpp"
#include "stereo_disparity.hpp"
#include "stereo_configuration.hpp"
#include "utils.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>


int main() {
    const std::string &outputFile {"config/stereo.yaml"};

    StereoCalibrator cal;
    //cal.calibrate(outputFile);
    // cal.saveCalibration(outputFile);

    StereoConfiguration config;
    if (!config.loadFromFile(outputFile)) {
        return -1;
    }
    StereoRectifier rec(config);
    StereoDisparity disparity_computer(config.Q);

    live_disparity_map();

    return EXIT_SUCCESS;
}