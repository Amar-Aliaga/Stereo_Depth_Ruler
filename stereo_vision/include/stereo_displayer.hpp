#pragma once

#include "stereo_calibrator.hpp"
#include "stereo_configuration.hpp"
#include "stereo_disparity.hpp"
#include "stereo_rectifier.hpp"

#include <utility>
#include <vector>
#include <memory>


struct MouseMat {
    std::shared_ptr<cv::Mat> raw_map;
    std::shared_ptr<cv::Mat> dis_map;
};

class StereoDisplayer {
    private:
        StereoConfiguration config;
        std::vector<cv::Point> clicked_points {};
        std::vector<std::pair<cv::Point, cv::Point>> points_history {};
        float dist {};
        std::vector<float> dist_vector {};
        MouseMat mouse_data;

    public:
        //StereoDisplayer(StereoConfiguration &config);
        StereoDisplayer() = default;

        static void MouseCallbackWrapper(int event, int x, int y, int flags, void *user_data);
        void onMouseMeasure(int event, int x, int y, int flags, void *user_data);
        void save_csvFile();
        void depth_coverage(const cv::Mat &mat);
        void live_disparity_map();

        //const MouseMat get_MouseMat() const;
};