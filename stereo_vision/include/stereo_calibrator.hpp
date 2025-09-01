#pragma once

#include <opencv2/opencv.hpp>
#include <string>

class StereoCalibrator {
private:
    const cv::Size boardSize {8, 6};
    static constexpr float squareSize {20.0f};
    static constexpr const char *left_frames_dir  {"/home/amar-aliaga/Desktop/left_frames/" };
    static constexpr const char *right_frames_dir {"/home/amar-aliaga/Desktop/right_frames/"};
    static constexpr const char *outputFile {"config/stereo.yaml"};

    cv::Mat cameraMatrixLeft, distCoeffsLeft;
    cv::Mat cameraMatrixRight, distCoeffsRight;
    cv::Mat R, T, E, F;
    cv::Mat R1, R2, P1, P2, Q;
    cv::Size imageSize;

public:
    StereoCalibrator();
    bool calibrate();
    void saveCalibration(const std::string& filename);
    void printCalibrationResults();
    bool run_calibration();

    const int getBoardSize_Width()    const noexcept;
    const int getBoardSize_Height()   const noexcept;
    const int getSquareSize()         const noexcept;

    const cv::Mat &getCameraMatrixLeft()   const noexcept;
    const cv::Mat &getDistCoeffsLeft()     const noexcept;
    const cv::Mat &getCameraMatrixRight()  const noexcept;
    const cv::Mat &getDistCoeffsRight()    const noexcept;

    const cv::Mat &getRotation()           const noexcept;
    const cv::Mat &getTranslation()        const noexcept;
    const cv::Mat &getRectificationLeft()  const noexcept;
    const cv::Mat &getRectificationRight() const noexcept;
    const cv::Mat &getProjectionLeft()     const noexcept;
    const cv::Mat &getProjectionRight()    const noexcept;
    const cv::Mat &getDisparityToDepth()   const noexcept;
};