#pragma once

#include "stereo_configuration.hpp"
#include <opencv2/opencv.hpp>
#include <string>

class StereoCalibrator {
private:
    const cv::Size boardSize {8, 6};
    static constexpr float squareSize {19.0f};
    static constexpr const char *left_frames_dir  {"/home/amar-aliaga/Desktop/left_frames/" };
    static constexpr const char *right_frames_dir {"/home/amar-aliaga/Desktop/right_frames/"};
    
    StereoConfiguration config;

public:
    StereoCalibrator();
    bool calibrate(const std::string &outputFile);
    void saveCalibration(const std::string& filename);
    void printCalibrationResults();
    bool run_calibration();

    const int getBoardSize_Width()    const noexcept;
    const int getBoardSize_Height()   const noexcept;
    const float getSquareSize()         const noexcept;
};
