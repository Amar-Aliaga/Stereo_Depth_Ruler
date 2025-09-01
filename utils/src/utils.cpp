#include "utils.hpp"

#include <opencv2/opencv.hpp>
#include "stereo_calibrator.hpp"
#include "stereo_disparity.hpp"
#include "stereo_rectifier.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <limits>

void save_frames() {
    cv::VideoCapture cap(2);

    if(!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return;
    }

    std::string left_directory  {"/home/amar-aliaga/Desktop/left_frames/" };
    std::string right_directory {"/home/amar-aliaga/Desktop/right_frames/"};

    int frame_count = 1;

    while(true) {
        cv::Mat frame;
        cap >> frame;

        if(frame.empty()) {
            std::cerr << "Error: Blank frame." << std::endl;
            break;
        }

        cv::Mat left_image = frame(cv::Rect(0, 0, frame.cols/2, frame.rows));
        cv::Mat right_image = frame(cv::Rect(frame.cols/2, 0, frame.cols/2, frame.rows));

        std::vector<cv::Point2f> left_corners, right_corners;
        bool found_left_board  = cv::findChessboardCornersSB( left_image,  cv::Size(8,6),  left_corners);
        bool found_right_board = cv::findChessboardCornersSB(right_image,  cv::Size(8,6), right_corners);

        cv::drawChessboardCorners(left_image, cv::Size(8,6), left_corners, found_left_board);
        cv::drawChessboardCorners(right_image, cv::Size(8,6), right_corners, found_right_board);

        cv::imshow("Left Camera", left_image);
        cv::imshow("Right Camera", right_image);

        char key = cv::waitKey(10);

        if(key == 27) break;
        else if(key == ' ') {
            std::string left_filename  = left_directory  + "left_"  + std::to_string(frame_count) + ".jpg";
            std::string right_filename = right_directory + "right_" + std::to_string(frame_count) + ".jpg";

            // std::vector<cv::Point2f> left_corners, right_corners;
            // bool found_left_board  = cv::findChessboardCornersSB( left_image,  cv::Size(8,6),  left_corners);
            // bool found_right_board = cv::findChessboardCornersSB(right_image,  cv::Size(8,6), right_corners);

            if(found_left_board && found_right_board) {
                cv::imwrite(left_filename,   left_image);
                cv::imwrite(right_filename, right_image);
                std::cout << "Saved: " << frame_count << std::endl;
                frame_count++;
            } else {
                std::cout << "Take the picture again." << std::endl;
            }
        }
    }

    cap.release();
    cv::destroyAllWindows();

}

void change_filename() {
    std::string folder_path = "/home/amar-aliaga/Desktop/calibration_frames/";

    int rcount = 1;
    int lcount = 1;
    int count_img = 1;
    for(auto &i : std::filesystem::directory_iterator(folder_path)) {
        if(i.is_regular_file()) {
            std::string lfilename, rfilename;
            if(count_img <= 27) {
                lfilename =  "leftImg_" + std::to_string(lcount++) + ".jpg";
                std::filesystem::path new_lpath = i.path().parent_path() / lfilename;
                std::filesystem::rename(i.path(), new_lpath);
            } else {
                rfilename = "rightImg_" + std::to_string(rcount++) + ".jpg";
                std::filesystem::path new_rpath = i.path().parent_path() / rfilename;
                std::filesystem::rename(i.path(), new_rpath);
            }
            count_img++;
        }
    }
    std::cout << "File names update" << std::endl;
}


void capture_frame(const std::string &output_file) {
    cv::VideoCapture cap(output_file);
    if(!cap.isOpened()) {
        std::cerr << "Camera not found" << std::endl;
        return;
    }
    while(true) {
        cv::Mat frame;
        cap >> frame;
        if(frame.empty()) {
            std::cerr << "Frame not found" << std::endl;
            break;
        }
        cv::imshow("Video", frame);

        char c = cv::waitKey(30);
        if(c == 27) break;
        else if(c == ' ') {
            cv::imwrite("/home/amar-aliaga/image1.jpg", frame);
            std::cout << "Frame has been saved" << std::endl;
        }
    }
    cap.release();
    cv::destroyAllWindows();
}


void image_desparity(const std::string &img_file) {
    StereoRectifier rec;
    cv::Mat left_img, right_img;
    rec.loadCalibration("config/stereo.yaml");
    cv::Mat frame = cv::imread(img_file, cv::IMREAD_COLOR);
    cv::Mat left = frame(cv::Rect(0, 0, frame.cols/2, frame.rows));
    cv::Mat right = frame(cv::Rect(frame.cols/2, 0, frame.cols/2, frame.rows));
    rec.rectify(left, right, left_img, right_img);
    //rec.drawEpipolarLines(left_img, right_img);


    StereoDisparity disparity (rec.getQ());
        
    cv::Mat disp = disparity.computeDisparity(left_img, right_img);
    cv::Mat dispNormalized;
    if (!disp.empty()) {
        double minVal, maxVal;
        cv::minMaxLoc(disp, &minVal, &maxVal);
    if (maxVal > minVal) {
        disp.convertTo(dispNormalized, CV_8U, 255.0 / (maxVal - minVal), -255.0 * minVal / (maxVal - minVal));
    }
}

    cv::imshow("Original Left", disp);
    cv::waitKey(0);
}