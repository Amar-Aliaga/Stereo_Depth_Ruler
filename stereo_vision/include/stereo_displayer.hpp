#pragma once

#include "stereo_calibrator.hpp"
#include "stereo_configuration.hpp"
#include "stereo_disparity.hpp"
#include "stereo_rectifier.hpp"

#include <utility>
#include <vector>
#include <memory>


struct MeasurementRecord {
    short image_index {};
    cv::Point point1 {};
    cv::Point point2 {};
    float distance {};
};


struct MouseMat {
    std::shared_ptr<cv::Mat> raw_map {nullptr};
    std::shared_ptr<cv::Mat> dis_map {nullptr};
};


class StereoDisplayer {
    private:
        StereoConfiguration config;

        std::vector<cv::Point> clicked_points {};
        std::vector<std::pair<cv::Point, cv::Point>> points_history {};
        //std::vector<std::pair<cv::Point, cv::Point>> last_image_points {};
        std::vector<MeasurementRecord> measurement_record;
        std::vector<float> dist_vector {};

        float dist {};
        short current_image_index = 1;

        MouseMat mouse_data {};

        cv::Mat depth_map, overlay, frozen;

    public:
        //StereoDisplayer(StereoConfiguration &config);
        StereoDisplayer() = default;

        void onMouseMeasure(int event, int x, int y, int flags, void *user_data);
        static void MouseCallbackWrapper(int event, int x, int y, int flags, void *user_data);

        void save_csvFile();

        void depth_coverage(const cv::Mat &mat);
        void show_disparity_overlay();

        void test();
        void test_mouse(const cv::Mat &frame);
};
