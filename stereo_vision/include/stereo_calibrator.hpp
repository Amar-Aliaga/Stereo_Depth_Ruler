#pragma once

#include "stereo_configuration.hpp"
#include "stereo_pipeline.hpp"

#include <opencv2/opencv.hpp>
#include <string>

class StereoCalibrator : public StereoPipeline {
private:
    const cv::Size boardSize {8, 6};
    static constexpr auto squareSize {19.0f};
    static constexpr const char *left_frames_dir  {"/home/amar-aliaga/Desktop/left_frames/" };
    static constexpr const char *right_frames_dir {"/home/amar-aliaga/Desktop/right_frames/"};
    
    StereoConfiguration config;

public:
    StereoCalibrator();
    bool calibrate(const std::string &outputFile);
    void saveCalibration(const std::string& filename);
    void printCalibrationResults();
    bool process() override;

    const int getBoardSize_Width()    const noexcept;
    const int getBoardSize_Height()   const noexcept;
    const float getSquareSize()       const noexcept;
    const StereoConfiguration get_config() const noexcept;
};
