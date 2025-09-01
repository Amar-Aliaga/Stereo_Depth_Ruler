#include "stereo_rectifier.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

StereoRectifier::StereoRectifier() = default;


bool StereoRectifier::loadCalibration(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if(!fs.isOpened()) {
        std::cerr << "Failed to open calibration file: " << filename << std::endl;
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
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;

    fs.release();

    if (cameraMatrixLeft.empty() || cameraMatrixRight.empty() || R1.empty() || Q.empty()) {
        std::cerr << "Error: Failed to load one or more matrices from YAML" << std::endl;
        return false;
    }

    cv::initUndistortRectifyMap(cameraMatrixLeft, distCoeffsLeft, R1, P1, imageSize, CV_16SC2, mapL1, mapL2);
    cv::initUndistortRectifyMap(cameraMatrixRight, distCoeffsRight, R2, P2, imageSize, CV_16SC2, mapR1, mapR2);
    std::cout << "Remapping tables created." << std::endl;

    return true;
}


void StereoRectifier::rectify(const cv::Mat &left_src, const cv::Mat &right_src, cv::Mat &left_dst, cv::Mat &right_dst) {
    if (left_src.empty() || right_src.empty()) {
        std::cerr << "Error: Input images are empty" << std::endl;
        return;
    }

     cv::Mat leftResized, rightResized;
    if (left_src.size() != imageSize || right_src.size() != imageSize) {
    std::cerr << "Warning: Resizing input images to match calibration size (" << imageSize << ")." << std::endl;
    cv::resize(left_src, leftResized, imageSize);
    cv::resize(right_src, rightResized, imageSize);
} else {
    left_src.copyTo(leftResized);
    right_src.copyTo(rightResized);
}


    cv::remap(leftResized,   left_dst, mapL1, mapL2, cv::INTER_LINEAR);
    cv::remap(rightResized, right_dst, mapR1, mapR2, cv::INTER_LINEAR);
    std::cout << "Images rectified successfully!" << std::endl;
}


void StereoRectifier::drawEpipolarLines(cv::Mat &rectifiedLeft, cv::Mat &rectifiedRight) {
    for (int y = 0; y < rectifiedLeft.rows; y += 30) {
        cv::line(rectifiedLeft, cv::Point(0, y), cv::Point(rectifiedLeft.cols, y),
                cv::Scalar(0, 255, 0), 1);
        cv::line(rectifiedRight, cv::Point(0, y), cv::Point(rectifiedRight.cols, y),
                cv::Scalar(0, 255, 0), 1);
    }
}


bool StereoRectifier::run_rectification() {
    if (!loadCalibration(outputFile)) {
        std::cerr << "Failed to load calibration!" << std::endl;
        return false;
    }

    cv::VideoCapture cap("/home/amar-aliaga/Desktop/my_video/output.mp4");
    if(!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return false;
    }

    //cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    //cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
    //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    double width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "Actual resolution: " << width << "x" << height << std::endl;

    while(true) {
        cv::Mat frame;
        cap >> frame;

        if(frame.empty()) {
           std::cout << "No frame has been found." << std::endl; 
        }

        cv::Mat left_image  = frame(cv::Rect(0, 0, frame.cols/2, frame.rows));
        cv::Mat right_image = frame(cv::Rect(frame.cols/2, 0, frame.cols/2, frame.rows));

        std::cout << "left image: " << left_image.size() << std::endl;
        std::cout << "Right image: " << right_image.size() << std::endl;

        cv::Mat left_res, right_res;
        rectify(left_image, right_image, left_res, right_res);
        drawEpipolarLines(left_res, right_res);
        char c = cv::waitKey(100);
        if(c == 27) break;
        else {
            cv::imshow("Left Rectified Image", left_res);
            cv::imshow("Right Rectified Image", right_res);
        }
    }
    return true;
}


const cv::Size &StereoRectifier::getImageSize() const noexcept { return imageSize; }
const cv::Mat  &StereoRectifier::getQ() const noexcept { return Q; }