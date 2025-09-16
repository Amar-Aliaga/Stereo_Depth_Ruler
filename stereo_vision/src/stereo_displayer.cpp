#include "stereo_displayer.hpp"
#include "stereo_calibrator.hpp"
#include "stereo_disparity.hpp"
#include "stereo_rectifier.hpp"
#include "stereo_configuration.hpp"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <limits>
#include <iomanip>
#include <utility>
#include <cmath>
#include <memory>
#include <fstream>


//StereoDisplayer::StereoDisplayer(StereoConfiguration &config) : config(config) {}


void StereoDisplayer::onMouseMeasure(int event, int x, int y, int flags, void *user_data) {
    if (event != cv::EVENT_LBUTTONDOWN || !(flags & cv::EVENT_FLAG_SHIFTKEY)) return; 
    MouseMat *disp = static_cast<MouseMat*>(user_data);
    if(!disp || !disp->raw_map || !disp->dis_map) return;

    std::shared_ptr<cv::Mat> rawMap = std::make_shared<cv::Mat>(*disp->raw_map);
    std::shared_ptr<cv::Mat> disMap = std::make_shared<cv::Mat>(*disp->dis_map);

    if (x < 0 || x >= rawMap->cols || y < 0 || y >= rawMap->rows) return;
    
    clicked_points.push_back(cv::Point(x, y));

    cv::circle(*disMap, clicked_points[0], 3.71, cv::Scalar(0,0,255), -1);
    cv::circle(*disMap, clicked_points[1], 3.71, cv::Scalar(0,0,255), -1);
    cv::imshow("Paused Image", *disMap);

    if (clicked_points.size() == 2) {
        cv::Vec3f xyz1 = rawMap->at<cv::Vec3f>(clicked_points[0].y, clicked_points[0].x);
        cv::Vec3f xyz2 = rawMap->at<cv::Vec3f>(clicked_points[1].y, clicked_points[1].x);
        cv::line(*disMap, clicked_points[0], clicked_points[1], cv::Scalar(255, 255 ,0), 1.53);
        cv::imshow("Paused Image", *disMap);
        dist = cv::norm(xyz1 - xyz2);
        points_history.push_back(std::make_pair(clicked_points[0], clicked_points[1]));
        dist_vector.push_back(dist);
        std::cout << "Distance: " << dist/10 << " cm" << std::endl;
        clicked_points.clear();
    }
}


void StereoDisplayer::MouseCallbackWrapper(int event, int x, int y, int flags, void *user_data) {
    auto* displayer = static_cast<StereoDisplayer*>(user_data);
    if(displayer) {
        displayer->onMouseMeasure(event, x, y, flags, &(displayer->mouse_data));
    }
}


void StereoDisplayer::save_csvFile() {
    size_t i{0}, j{0};
        std::ofstream csvFile("/home/amar-aliaga/Desktop/wayland/wayland_thirdProject/results/measurements.csv");
        if(!csvFile.is_open()) {
            std::cerr << "Failed to create CSV file!" << std::endl;
            return;
        }
        csvFile << std::right << "First_point, " << std::setw(4) << "  Second_point, " << std::setw(4) << "Distance\n";
        csvFile << std::fixed << std::setprecision(5);

        while(i < points_history.size() && j < dist_vector.size()) {
            std::cout << "First point: " << points_history[i].first << std::endl;
            std::cout << "Second point: " << points_history[i].second << std::endl;
            std::cout << "Distance: " << dist_vector[j] / 10 << "cm" << std::endl;

            csvFile << std::right << points_history[i].first << ", " << std::setw(4)
                    << points_history[i].second << ", " << std::setw(6)
                    << dist_vector[j]/10 <<  " cm" << std::setw(4)  << "\n";
            i++;
            j++;
        }
        std::cout << std::endl;

        csvFile.close();

        std::cout << "Measurements saved to /results/measurements.csv" << std::endl;
}


void StereoDisplayer::depth_coverage(const cv::Mat &mat) {
    int counter {0};
    const int total_pixels = mat.rows * mat.cols;
    for(int i = 0; i<mat.rows; ++i) {
        for(int j = 0; j<mat.cols; ++j) {
            cv::Vec3f conf_pixel = mat.at<cv::Vec3f>(i, j);
            if(conf_pixel[2] >= 0.0 && conf_pixel[2] <= 12000.0 && !std::isnan(conf_pixel[2])) { // && conf_pixel[2] <= 1.0
                counter++;
            }
        }
    }
    double coverage_percentage = (static_cast<double>(counter) / total_pixels) * 100;
    std::cout << "Depth Coverage: " << coverage_percentage << std::endl;
}


void StereoDisplayer::show_disparity_overlay() {
    const std::string &s {"/home/amar-aliaga/Desktop/my_video/output.mp4"};
    const std::string &v {"/home/amar-aliaga/Downloads/cam.mp4"};

    if (!config.loadFromFile("config/stereo.yaml")) {
        std::cerr << "Error: Could not load configuration file." << std::endl;
        return;
    }
    StereoRectifier rectifier(config);
    StereoDisparity disparity_computer(config.Q);

    cv::VideoCapture cap(s);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video file." << std::endl;
        return;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    int w = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int h = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    std::cout << "Res: " << w << "x" << h << std::endl;

    while (true) {
        cv::Mat frame;

        cap >> frame;
        if (frame.empty()) {
            std::cout << "End of video." << std::endl;
            break;
        }


        cv::Mat left_raw = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
        cv::Mat right_raw = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

        cv::Mat left_rect, right_rect;
        rectifier.rectify(left_raw, right_raw, left_rect, right_rect);

        cv::Mat disp_float = disparity_computer.computeDisparity(left_rect, right_rect);
        depth_map = disparity_computer.computeDepth(disp_float);

        cv::Mat display_depth = disparity_computer.show_depthMap(depth_map);
        cv::Mat display_disparity = disparity_computer.show_disparityMap(disp_float);

        cv::Mat disparity_heatmap;
        cv::applyColorMap(display_disparity, disparity_heatmap, cv::COLORMAP_JET);

        cv::resize(disparity_heatmap, disparity_heatmap, display_depth.size());
        cv::resize(left_rect, left_rect, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

        cv::addWeighted(left_rect, 0.7, disparity_heatmap, 0.3, 0, overlay);

        cv::imshow("Left Rectified", left_rect);
        cv::moveWindow("Left Rectified", 2200, 300);
        cv::imshow("Depth Map", display_depth);
        cv::moveWindow("Depth Map", 3100, 300);
        cv::imshow("Left: rectified image + disparity overlay", overlay);
        cv::moveWindow("Left: rectified image + disparity overlay", 2200, 400 + left_rect.rows);

        depth_coverage(depth_map);

        int key = cv::waitKey(1);
        switch(key) {
            case 27:
                return;
            case 102:
                frozen = overlay.clone();
                cv::imshow("Paused Image", frozen);
                cv::moveWindow("Paused Image", 3100, 400 + left_rect.rows);
                test_mouse(depth_map);
                break;
            default: 
                std::cout << "Press either Esc or f." << std::endl;
                break;
        }
    }
}


void StereoDisplayer::test_mouse(const cv::Mat &depth_map) {
    if (depth_map.empty()) {
        std::cerr << "Could not load depth map for measurement." << std::endl;
        return;
    }

    MouseMat mouse_data;
    mouse_data.raw_map = std::make_shared<cv::Mat>(depth_map.clone());
    mouse_data.dis_map = std::make_shared<cv::Mat>(frozen.clone()); 

    this->mouse_data = mouse_data;

    cv::imshow("Paused Image", *mouse_data.dis_map);
    cv::setMouseCallback("Paused Image", StereoDisplayer::MouseCallbackWrapper, this);

    while (true) {
        int key = cv::waitKey(1);
        if (key == 102 || key == 70) break;
        else if(key == 97 || key == 65) {
            return;
        } else if(key == 115) {
            save_csvFile();
            std::cout << "File has been saved." << std::endl;
        } else if(key == 114) {
            clicked_points.clear();
            points_history.clear();
            dist_vector.clear();
            std::cout << "The values have been rested." << std::endl;

            mouse_data.dis_map = std::make_shared<cv::Mat>(frozen.clone());
            this->mouse_data = mouse_data;
            cv::imshow("Paused Image", *mouse_data.dis_map);
        }
    }
    cv::destroyWindow("Paused Image");
}
