#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

#include <string>
#include <memory>

class StereoPipeline {
    public:
        const std::string outputFile {"config/stereo.yaml"};
        cv::Mat depth_map, overlay, frozen;

        virtual bool process() = 0;
        virtual ~StereoPipeline() = default;
};