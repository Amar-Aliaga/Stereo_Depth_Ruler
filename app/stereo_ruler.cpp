// #include <opencv2/opencv.hpp>
// #include <opencv2/ximgproc.hpp>
#include "stereo_calibrator.hpp"
#include "stereo_rectifier.hpp"
#include "stereo_disparity.hpp"
#include "utils.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>


int main() {

    StereoRectifier rec;
    //rec.loadCalibration("config/stereo.yaml")
    live_disparity_map();

    return EXIT_SUCCESS;
}