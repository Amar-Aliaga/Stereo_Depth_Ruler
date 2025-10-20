#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

#include "stereo_configuration.hpp"

#include <string>
#include <memory>

class StereoPipeline {
    protected:
        const std::string outputFile {"config/stereo.yaml"};
        cv::Mat depth_map, overlay, frozen;

    public:
        virtual bool process() = 0;
        //virtual void display() = 0;
        virtual ~StereoPipeline() = default;
};
