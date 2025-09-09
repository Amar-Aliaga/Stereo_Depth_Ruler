#include "stereo_rectifier.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

StereoRectifier::StereoRectifier(const StereoConfiguration &config) : config(config) {
    cv::initUndistortRectifyMap(config.cameraMatrixLeft, config.distCoeffsLeft, config.R1, 
                                config.P1, config.imageSize, CV_16SC2, mapL1, mapL2);

    cv::initUndistortRectifyMap(config.cameraMatrixRight, config.distCoeffsRight, config.R2, 
                                config.P2, config.imageSize, CV_16SC2, mapR1, mapR2);

    std::cout << "Remapping tables created." << std::endl;
}


void StereoRectifier::rectify(const cv::Mat &left_src, const cv::Mat &right_src, cv::Mat &left_dst, cv::Mat &right_dst) {
    if (left_src.empty() || right_src.empty()) {
        std::cerr << "Error: Input images are empty" << std::endl;
        return;
    }

     cv::Mat leftResized, rightResized;
    if (left_src.size() != config.imageSize || right_src.size() != config.imageSize) {
    std::cerr << "Warning: Resizing input images to match calibration size (" << config.imageSize << ")." << std::endl;
    cv::resize(left_src, leftResized, config.imageSize);
    cv::resize(right_src, rightResized, config.imageSize);
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
    const std::string &outputFile("config/stereo.yaml");

    // if (!config.isValid(outputFile)) {
    //     std::cerr << "Failed to load calibration!" << std::endl;
    //     return false;
    // }

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
