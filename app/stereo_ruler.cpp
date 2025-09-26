// #include <opencv2/opencv.hpp>
// #include <opencv2/ximgproc.hpp>
#include "stereo_calibrator.hpp"
#include "stereo_rectifier.hpp"
#include "stereo_disparity.hpp"
#include "stereo_configuration.hpp"
#include "stereo_displayer.hpp"
#include "stereo_pipeline.hpp"

#include "utils.hpp"
#include "helper.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <memory>

int main() {
    const std::string &outputFile {"config/stereo.yaml"};

    std::unique_ptr<StereoPipeline> pipeline = std::make_unique<StereoCalibrator>();
    pipeline->process();

    // StereoConfiguration config;
    // if (!config.loadFromFile("config/stereo.yaml")) {
    //     std::cerr << "Failed to load config!" << std::endl;
    //     return -1;
    // }

    // // std::unique_ptr<StereoPipeline> pipeline1 = std::make_unique<StereoRectifier>(config);
    // // pipeline1->process();

    // std::unique_ptr<StereoPipeline> pipeline2 = std::make_unique<StereoDisparity>(config.Q);
    // pipeline2->process();

    //StereoCalibrator cal;
    //cal.calibrate(outputFile);

    // StereoConfiguration config;
    // if (!config.loadFromFile(outputFile)) {
    //     return -1;
    // }
    // StereoRectifier rec(config);
    // StereoDisparity disparity_computer(config.Q);

    //zed_footage();
    // std::unique_ptr<StereoPipeline> sd = std::make_unique<StereoDisplayer>();;
    // sd->process();
    //sd.image_depth("/home/amar-aliaga/stereo_image2.png");

    return EXIT_SUCCESS;
}

/*
cppreference.com: Constructors
cppreference.com: Initialization
C++ Books
Effective C++ by Scott Meyers (see the item about member initialization)
The C++ Programming Language by Bjarne Stroustrup
Online Tutorials
LearnCpp.com: Member initialization list
Stack Overflow: Why do I need to use an initializer list in C++?

*/