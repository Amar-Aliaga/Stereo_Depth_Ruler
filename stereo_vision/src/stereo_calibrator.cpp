#include "stereo_calibrator.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>


StereoCalibrator::StereoCalibrator() = default;

bool StereoCalibrator::calibrate(const std::string &outputFile) {
    std::vector<std::vector<cv::Point2f>> imagePointsLeft, imagePointsRight;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }

    std::vector<std::string> left_files, right_files;
    int successfulPairs = 0;
    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH + 
                        cv::CALIB_CB_NORMALIZE_IMAGE + 
                        cv::CALIB_CB_FAST_CHECK;

    for(const auto& entry : std::filesystem::directory_iterator(left_frames_dir)) {
        left_files.push_back(entry.path().string());
    }

    for(const auto& entry : std::filesystem::directory_iterator(right_frames_dir)) {
            right_files.push_back(entry.path().string());
    }

    std::sort( left_files.begin(),  left_files.end());
    std::sort(right_files.begin(), right_files.end());

    
    size_t max_pairs = std::min(left_files.size(), right_files.size());

    for (int i = 0; i < max_pairs; i++) { 
        cv::Mat leftImg  = cv::imread( left_files[i], cv::IMREAD_GRAYSCALE);
        cv::Mat rightImg = cv::imread(right_files[i], cv::IMREAD_GRAYSCALE);

        if (leftImg.empty() || rightImg.empty()) {
            std::cout << "Skipping pair " << i << " - images not found" << std::endl;
            continue;
        }
        
        if (config.imageSize.width == 0) {
          config.imageSize = leftImg.size();
        }

        std::vector<cv::Point2f> cornersLeft, cornersRight;
        bool foundLeft  = cv::findChessboardCorners(leftImg, boardSize, cornersLeft, flags);
        bool foundRight = cv::findChessboardCorners(rightImg, boardSize, cornersRight, flags);

        std::cout << "Pair " << i << ": left=" << foundLeft << ", right=" << foundRight << std::endl;

        if (foundLeft && foundRight) {
            cv::cornerSubPix(leftImg, cornersLeft, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
            cv::cornerSubPix(rightImg, cornersRight, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));

            imagePointsLeft.push_back(cornersLeft);
            imagePointsRight.push_back(cornersRight);
            objectPoints.push_back(obj);
            successfulPairs++;

            cv::Mat displayLeft, displayRight;
            cv::cvtColor(leftImg,  displayLeft, cv::COLOR_GRAY2BGR);
            cv::cvtColor(rightImg, displayRight, cv::COLOR_GRAY2BGR);
            
            cv::drawChessboardCorners(displayLeft, boardSize, cornersLeft, foundLeft);
            cv::drawChessboardCorners(displayRight, boardSize, cornersRight, foundRight);
            
            cv::imshow("Left Camera", displayLeft);
            cv::imshow("Right Camera", displayRight);
            cv::waitKey(21); 
        }
    }

    std::cout << "Found " << successfulPairs << " valid stereo pairs" << std::endl;

    if (successfulPairs < 20) {
        std::cerr << "Error: Need at least 20 stereo pairs for calibration" << std::endl;
        return false;
    }

    std::vector<cv::Mat> rvecsLeft, tvecsLeft;
    std::vector<cv::Mat> rvecsRight, tvecsRight;

    double rmsLeft  = cv::calibrateCamera(objectPoints, imagePointsLeft, config.imageSize,
        config.cameraMatrixLeft, config.distCoeffsLeft, rvecsLeft, tvecsLeft);
    double rmsRight = cv::calibrateCamera(objectPoints, imagePointsRight, config.imageSize,
        config.cameraMatrixRight, config.distCoeffsRight, rvecsRight, tvecsRight);

    std::cout << "Left camera RMS error: " << rmsLeft << std::endl;
    std::cout << "Right camera RMS error: " << rmsRight << std::endl;

    double rmsStereo = cv::stereoCalibrate(
        objectPoints, imagePointsLeft, imagePointsRight,
        config.cameraMatrixLeft, config.distCoeffsLeft,
        config.cameraMatrixRight, config.distCoeffsRight,
        config.imageSize, config.R, config.T, config.E, config.F, 0,
        //cv::CALIB_FIX_INTRINSIC + cv::CALIB_USE_INTRINSIC_GUESS,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5)
    );

    std::cout << "Stereo calibration RMS error: " << rmsStereo << std::endl;

    cv::stereoRectify(
        config.cameraMatrixLeft, config.distCoeffsLeft,
        config.cameraMatrixRight, config.distCoeffsRight,
        config.imageSize, config.R, config.T, config.R1, config.R2, config.P1, config.P2, config.Q,
        cv::CALIB_ZERO_DISPARITY, 0, config.imageSize
    );

    saveCalibration(outputFile);

    return true;
}


void StereoCalibrator::saveCalibration(const std::string &filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        
    if (fs.isOpened()) {
        fs << "imageWidth" << config.imageSize.width;
        fs << "imageHeight" << config.imageSize.height;
        fs << "cameraMatrixLeft" << config.cameraMatrixLeft;
        fs << "distCoeffsLeft" << config.distCoeffsLeft;
        fs << "cameraMatrixRight" << config.cameraMatrixRight;
        fs << "distCoeffsRight" << config.distCoeffsRight;
        fs << "R" << config.R;
        fs << "T" << config.T;
        fs << "E" << config.E;
        fs << "F" << config.F;
        fs << "R1" << config.R1;
        fs << "R2" << config.R2;
        fs << "P1" << config.P1;
        fs << "P2" << config.P2;
        fs << "Q" << config.Q;
        
        fs.release();
        std::cout << "Calibration saved to " << filename << std::endl;
    } else {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
    }
}


void StereoCalibrator::printCalibrationResults() {
    std::cout << "\n=== CALIBRATION RESULTS ===" << std::endl;
    std::cout << "Left Camera Matrix:\n" << config.cameraMatrixLeft << std::endl;
    std::cout << "Left Distortion Coefficients:\n" << config.distCoeffsLeft << std::endl;
    std::cout << "Right Camera Matrix:\n" << config.cameraMatrixRight << std::endl;
    std::cout << "Right Distortion Coefficients:\n" << config.distCoeffsRight << std::endl;
    std::cout << "Rotation Matrix:\n" << config.R << std::endl;
    std::cout << "Translation Vector:\n" << config.T << std::endl;
    std::cout << "Essential Matrix:\n" << config.E << std::endl;
    std::cout << "Fundamental Matrix:\n" << config.F << std::endl;
}


bool StereoCalibrator::run_calibration() {

    const std::string &outputFile("config/stereo.yaml");
    
    std::cout << "Starting stereo calibration..." << std::endl;
    std::cout << "Pattern: " << getBoardSize_Width() << "x" << getBoardSize_Height() << " inner corners" << std::endl;
    std::cout << "Square size: " << getSquareSize() << " mm" << std::endl;

    if (calibrate(outputFile)) {
        std::cout << "Calibration successful!" << std::endl;
        
        saveCalibration(outputFile);
        
        std::cout << "\nCalibration completed successfully!" << std::endl;
        std::cout << "Results saved to: " << outputFile << std::endl;
    } else {
        std::cerr << "Calibration failed!" << std::endl;
        return false;
    }
    return true;
}


const int StereoCalibrator::getBoardSize_Width()    const noexcept { return boardSize.width;   }
const int StereoCalibrator::getBoardSize_Height()   const noexcept { return boardSize.height;  }
const float StereoCalibrator::getSquareSize()         const noexcept { return squareSize;        }
