#include "stereo_configuration.hpp"
#include <iostream>

bool StereoConfiguration::loadFromFile(const std::string &filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }

    int calibWidth{0}, calibHeight{0};
    fs["imageWidth"] >> calibWidth;
    fs["imageHeight"] >> calibHeight;

    if (calibWidth > 0 && calibHeight > 0) {
        imageSize = cv::Size(calibWidth, calibHeight);
    } else {
        std::cerr << "Error: Invalid image size in YAML (width=" << calibWidth << ", height=" << calibHeight << ")" << std::endl;
        return false;
    }  

    fs["cameraMatrixLeft"] >> cameraMatrixLeft;
    fs["distCoeffsLeft"] >> distCoeffsLeft;
    fs["cameraMatrixRight"] >> cameraMatrixRight;
    fs["distCoeffsRight"] >> distCoeffsRight;
    fs["R"] >> R;
    fs["T"] >> T;
    fs["E"] >> E;
    fs["F"] >> F;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;

    fs.release();

    if (cameraMatrixLeft.empty() || cameraMatrixRight.empty() ||
        R1.empty() || R2.empty() || P1.empty() || P2.empty() || Q.empty()) {
        std::cerr << "Error: Failed to load one or more essential matrices from " << filename << std::endl;
        return false;
    }

    std::cout << "Stereo configuration loaded successfully from " << filename << std::endl;

    return true;
}


bool StereoConfiguration::saveToFile(const std::string &filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        return false;
    }
    
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
    fs << "imageSize" << imageSize;
    
    fs.release();
    return true;
}


bool StereoConfiguration::isValid() const {
    return !cameraMatrixLeft.empty() && !cameraMatrixRight.empty() && 
        !R.empty() && !T.empty() && !Q.empty();
}
