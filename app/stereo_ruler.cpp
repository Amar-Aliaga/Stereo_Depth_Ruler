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

    std::vector<std::unique_ptr<StereoPipeline>> pipelines;
    auto &config = StereoConfiguration::getConfig();

    // pipelines.push_back(std::make_unique<StereoCalibrator>());
    // pipelines.push_back(std::make_unique<StereoRectifier>(config));
    // pipelines.push_back(std::make_unique<StereoDisparity>(config.Q));
    // pipelines.push_back(std::make_unique<StereoDisplayer>());

    // for (auto& pipeline : pipelines) {
    //     pipeline->process();
    // }

    // std::unique_ptr<StereoPipeline> pipeline = std::make_unique<StereoCalibrator>();
    // pipeline->process();



    // std::unique_ptr<StereoPipeline> pipeline1 = std::make_unique<StereoRectifier>(config);
    // pipeline1->process();

    // std::unique_ptr<StereoPipeline> pipeline2 = std::make_unique<StereoDisparity>(config.Q);
    // if(!pipeline2->process()) {
    //     std::cerr << "Null" << std::endl;
    //     return -1;
    // }


    std::unique_ptr<StereoPipeline> sd = std::make_unique<StereoDisplayer>();
    sd->process();
    //sd.image_depth("/home/amar-aliaga/stereo_image2.png");

    // PointCloud pcl;
    // pcl.show_live_pointCloud();

    return EXIT_SUCCESS;
}
