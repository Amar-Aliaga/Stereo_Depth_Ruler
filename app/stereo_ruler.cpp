#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

#include "stereo_calibrator.hpp"
#include "stereo_rectifier.hpp"
#include "stereo_disparity.hpp"
#include "stereo_configuration.hpp"
#include "stereo_displayer.hpp"
#include "stereo_pipeline.hpp"

#include "pcd_write.hpp"

#include "utils.hpp"
#include "helper.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <memory>

int main() {
    // std::unique_ptr<StereoPipeline> pipeline = std::make_unique<StereoCalibrator>();
    // pipeline->process();


    // auto &config = StereoConfiguration::getConfig();

    // std::unique_ptr<StereoPipeline> pipeline1 = std::make_unique<StereoRectifier>(config);
    // pipeline1->process();

    // std::unique_ptr<StereoPipeline> pipeline2 = std::make_unique<StereoDisparity>(config.Q);
    // if(!pipeline2->process()) {
    //     std::cerr << "Null" << std::endl;
    //     return -1;
    // }


    //zed_footage();
    std::unique_ptr<StereoPipeline> sd = std::make_unique<StereoDisplayer>();;
    sd->process();
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

/*
int main() {
    const std::string configPath = "config/stereo.yaml";
    auto& config = StereoConfiguration::getConfig();

    // Get initial last write time
    auto last_write = std::filesystem::last_write_time(configPath);

    while (true) {
        // Check for file modification every second
        std::this_thread::sleep_for(std::chrono::seconds(1));
        auto current_write = std::filesystem::last_write_time(configPath);

        if (current_write != last_write) {
            std::cout << "Config file changed! Reloading..." << std::endl;
            config.loadFromFile(configPath);
            last_write = current_write;
        }

        // ... your main loop logic ...
        // For demonstration, break after some time or on user input
    }

    return 0;
}*/