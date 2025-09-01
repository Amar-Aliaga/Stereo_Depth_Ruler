#include "stereo_calibrator.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>


StereoCalibrator::StereoCalibrator() = default;

bool StereoCalibrator::calibrate() {
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
        
        if (this->imageSize.width == 0) {
          this->imageSize = leftImg.size();
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

    double rmsLeft = cv::calibrateCamera(objectPoints, imagePointsLeft, imageSize,
        cameraMatrixLeft, distCoeffsLeft, rvecsLeft, tvecsLeft);
    double rmsRight = cv::calibrateCamera(objectPoints, imagePointsRight, imageSize,
        cameraMatrixRight, distCoeffsRight, rvecsRight, tvecsRight);

    std::cout << "Left camera RMS error: " << rmsLeft << std::endl;
    std::cout << "Right camera RMS error: " << rmsRight << std::endl;

    double rmsStereo = cv::stereoCalibrate(
        objectPoints, imagePointsLeft, imagePointsRight,
        cameraMatrixLeft, distCoeffsLeft,
        cameraMatrixRight, distCoeffsRight,
        imageSize, R, T, E, F,
        cv::CALIB_FIX_INTRINSIC + cv::CALIB_USE_INTRINSIC_GUESS,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5)
    );

    std::cout << "Stereo calibration RMS error: " << rmsStereo << std::endl;

    T = -T;
    R = R.t();

    cv::stereoRectify(
        cameraMatrixLeft, distCoeffsLeft,
        cameraMatrixRight, distCoeffsRight,
        imageSize, R, T, R1, R2, P1, P2, Q,
        cv::CALIB_ZERO_DISPARITY, 0, imageSize
    );
    return true;
}


void StereoCalibrator::saveCalibration(const std::string &filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        
    if (fs.isOpened()) {
        fs << "imageWidth" << imageSize.width;
        fs << "imageHeight" << imageSize.height;
        fs << "cameraMatrixLeft" << cameraMatrixLeft;
        fs << "distCoeffsLeft" << distCoeffsLeft;
        fs << "cameraMatrixRight" << cameraMatrixRight;
        fs << "distCoeffsRight" << distCoeffsRight;
        fs << "R" << R;
        fs << "T" << T;
        fs << "E" << E;
        fs << "F" << F;
        fs << "R1" << R1;
        fs << "R2" << R2;
        fs << "P1" << P1;
        fs << "P2" << P2;
        fs << "Q" << Q;
        
        fs.release();
        std::cout << "Calibration saved to " << filename << std::endl;
    } else {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
    }
}


void StereoCalibrator::printCalibrationResults() {
    std::cout << "\n=== CALIBRATION RESULTS ===" << std::endl;
    std::cout << "Left Camera Matrix:\n" << cameraMatrixLeft << std::endl;
    std::cout << "Left Distortion Coefficients:\n" << distCoeffsLeft << std::endl;
    std::cout << "Right Camera Matrix:\n" << cameraMatrixRight << std::endl;
    std::cout << "Right Distortion Coefficients:\n" << distCoeffsRight << std::endl;
    std::cout << "Rotation Matrix:\n" << R << std::endl;
    std::cout << "Translation Vector:\n" << T << std::endl;
    std::cout << "Essential Matrix:\n" << E << std::endl;
    std::cout << "Fundamental Matrix:\n" << F << std::endl;
}


bool StereoCalibrator::run_calibration() {
    
    std::cout << "Starting stereo calibration..." << std::endl;
    std::cout << "Pattern: " << getBoardSize_Width() << "x" << getBoardSize_Height() << " inner corners" << std::endl;
    std::cout << "Square size: " << getSquareSize() << " mm" << std::endl;

    if (calibrate()) {
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
const int StereoCalibrator::getSquareSize()         const noexcept { return squareSize;        }

const cv::Mat &StereoCalibrator::getCameraMatrixLeft()   const noexcept { return cameraMatrixLeft;  }
const cv::Mat &StereoCalibrator::getDistCoeffsLeft()     const noexcept { return distCoeffsLeft;    }
const cv::Mat &StereoCalibrator::getCameraMatrixRight()  const noexcept { return cameraMatrixRight; }
const cv::Mat &StereoCalibrator::getDistCoeffsRight()    const noexcept { return distCoeffsRight;   }

const cv::Mat &StereoCalibrator::getRotation()           const noexcept { return R;  }
const cv::Mat &StereoCalibrator::getTranslation()        const noexcept { return T;  }
const cv::Mat &StereoCalibrator::getRectificationLeft()  const noexcept { return R1; }
const cv::Mat &StereoCalibrator::getRectificationRight() const noexcept { return R2; }
const cv::Mat &StereoCalibrator::getProjectionLeft()     const noexcept { return P1; }
const cv::Mat &StereoCalibrator::getProjectionRight()    const noexcept { return P2; }
const cv::Mat &StereoCalibrator::getDisparityToDepth()   const noexcept { return Q;  }