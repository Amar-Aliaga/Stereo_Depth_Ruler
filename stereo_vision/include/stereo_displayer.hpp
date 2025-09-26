#pragma once

#include "stereo_calibrator.hpp"
#include "stereo_configuration.hpp"
#include "stereo_disparity.hpp"
#include "stereo_rectifier.hpp"
#include "stereo_pipeline.hpp"

#include <utility>
#include <vector>
#include <memory>


struct MeasurementRecord {
    short image_index {};
    cv::Point point1  {};
    cv::Point point2  {};
    float distance    {};
};


struct MouseMat {
    std::shared_ptr<cv::Mat> raw_map {nullptr};
    std::shared_ptr<cv::Mat> dis_map {nullptr};
};


class StereoDisplayer : public StereoPipeline {
    private:
        StereoConfiguration config;

        std::vector<cv::Point> clicked_points {};
        std::vector<std::pair<cv::Point, cv::Point>> points_history {};
        std::vector<MeasurementRecord> measurement_record {};
        std::vector<float> dist_vector {};

        float dist {};
        short current_image_index = 1;

        MouseMat mouse_data {};

    public:
        //StereoDisplayer(StereoConfiguration &config);
        StereoDisplayer() = default;

        void onMouseMeasure(int event, int x, int y, int flags, void *user_data);
        static void MouseCallbackWrapper(int event, int x, int y, int flags, void *user_data);

        void save_csvFile();

        void depth_coverage(const cv::Mat &mat);
        bool process() override;

        void measure_points(const cv::Mat &frame);

        void image_depth(const std::string &path);
};
