// #include <opencv2/opencv.hpp>
// #include <opencv2/ximgproc.hpp>
#include "stereo_calibrator.hpp"
#include "stereo_rectifier.hpp"
#include "stereo_disparity.hpp"
#include "stereo_configuration.hpp"
#include "stereo_displayer.hpp"
#include "utils.hpp"
#include "helper.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>


int main() {
    const std::string &outputFile {"config/stereo.yaml"};

    StereoCalibrator cal;
    //cal.calibrate(outputFile);

    StereoConfiguration config;
    if (!config.loadFromFile(outputFile)) {
        return -1;
    }
    StereoRectifier rec(config);
    StereoDisparity disparity_computer(config.Q);

    //zed_footage();
    StereoDisplayer sd;
    sd.show_disparity_overlay();
    //sd.image_depth("/home/amar-aliaga/rama_img.jpg");

    return EXIT_SUCCESS;
}
