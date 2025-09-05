#include "utils.hpp"

#include <opencv2/opencv.hpp>
#include "stereo_calibrator.hpp"
#include "stereo_disparity.hpp"
#include "stereo_rectifier.hpp"
#include "stereo_configuration.hpp"
#include <sl/Camera.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <limits>

static std::vector<cv::Point> clicked_points;
static cv::Mat global_depth_map;

// Mouse callback for measuring distance
void onMouseMeasure(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN && !global_depth_map.empty()) {
        clicked_points.push_back(cv::Point(x, y));
        if (clicked_points.size() == 2) {
            cv::Vec3f xyz1 = global_depth_map.at<cv::Vec3f>(clicked_points[0]);
            cv::Vec3f xyz2 = global_depth_map.at<cv::Vec3f>(clicked_points[1]);
            double dist = cv::norm(xyz1 - xyz2);
            std::cout << "Distance: " << dist << " mm" << std::endl;
            clicked_points.clear();
        }
    }
}


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
    StereoConfiguration config;
    
    if (!config.loadFromFile("config/stereo.yaml")) {
        return;
    }
    StereoRectifier rec(config);
    cv::Mat left_img, right_img;
    //rec.loadCalibration("config/stereo.yaml");
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

void zed_footage() {
    sl::Camera zed;

    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 30;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_params.coordinate_units = sl::UNIT::METER;

    sl::ERROR_CODE err = zed.open(init_params);
    if(err != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "Error" << std::endl;
        return;
    }

    sl::RuntimeParameters runtime_params;

    sl::Mat zed_right_image, zed_left_image;
    cv::Mat cv_right_image, cv_left_image;

    while(true) {
        if(zed.grab(runtime_params) == sl::ERROR_CODE::SUCCESS) {
            zed.retrieveImage(zed_left_image, sl::VIEW::LEFT);
            zed.retrieveImage(zed_right_image, sl::VIEW::RIGHT);

            cv_left_image  = cv::Mat((int)zed_left_image.getHeight(),  (int)zed_left_image.getWidth(),  CV_8UC4, zed_left_image.getPtr<sl::uchar1>(sl::MEM::CPU));
            cv_right_image = cv::Mat((int)zed_right_image.getHeight(), (int)zed_right_image.getWidth(), CV_8UC4, zed_right_image.getPtr<sl::uchar1>(sl::MEM::CPU));

            cv::imshow("ZED Left Camera", cv_left_image);
            cv::imshow("ZED Right Camera", cv_right_image);

            char key = cv::waitKey(10);
            if (key == 27) { 
                break;
            }
        }
    }

}


void live_disparity_map() {
        const std::string &s {"/home/amar-aliaga/Desktop/my_video/output.mp4"};

        StereoConfiguration config;

        if (!config.loadFromFile("config/stereo.yaml")) {
            return;
        }
        StereoRectifier rectifier(config);

        StereoDisparity disparity_computer(config.Q);

        cv::VideoCapture cap(2);
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open video file." << std::endl;
            return;
        }

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

            cv::Mat depth_map = disparity_computer.computeDepth(disp_float);

            cv::Mat depthZ;
            if (depth_map.channels() == 3) {
                cv::extractChannel(depth_map, depthZ, 2); 
            } else {
                depthZ = depth_map; 
            }

//             double zmin = 500.0, zmax = 5000.0;
//             cv::Mat z_valid = (depthZ > 0) & (depthZ < 10000) & (depthZ == depthZ);

//             double zmin_raw, zmax_raw;
//             cv::minMaxLoc(depthZ, &zmin_raw, &zmax_raw, nullptr, nullptr, z_valid);

//             double alpha = 0.1;
//             zmin = (1.0 - alpha) * zmin + alpha * zmin_raw;
//             zmax = (1.0 - alpha) * zmax + alpha * zmax_raw;

//             cv::Mat depth8u, depth_vis8u;
// ;
//             depthZ.convertTo(depth8u, CV_8U, 255.0 / (zmax - zmin), -255.0 * zmin / (zmax - zmin));

//             cv::applyColorMap(depth8u, depth_vis8u, cv::COLORMAP_TURBO);
// ...existing code...
static double zmin_smooth = 1000.0, zmax_smooth = 2000.0;

// Mask valid depth values: positive, finite, and not huge
cv::Mat z_valid = (depthZ > 0) & (depthZ < 10000) & (depthZ == depthZ);

double zmin_raw = 0, zmax_raw = 0;
cv::minMaxLoc(depthZ, &zmin_raw, &zmax_raw, nullptr, nullptr, z_valid);

// If no valid pixels, use defaults to avoid division by zero
if (!(zmax_raw > zmin_raw)) {
    zmin_raw = 1000.0;
    zmax_raw = 2000.0;
}

double alpha = 0.1;
zmin_smooth = (1.0 - alpha) * zmin_smooth + alpha * zmin_raw;
zmax_smooth = (1.0 - alpha) * zmax_smooth + alpha * zmax_raw;

// Clamp smoothing range to reasonable values
zmin_smooth = std::max(0.0, std::min(zmin_smooth, 10000.0));
zmax_smooth = std::max(zmin_smooth + 1.0, std::min(zmax_smooth, 10000.0)); // ensure zmax > zmin

cv::Mat depth8u;
depthZ.convertTo(depth8u, CV_8U, 255.0 / (zmax_smooth - zmin_smooth), -255.0 * zmin_smooth / (zmax_smooth - zmin_smooth));

cv::Mat depth_vis8u;
cv::applyColorMap(depth8u, depth_vis8u, cv::COLORMAP_TURBO);
// ...existing code...

            //depthZ.convertTo(depth_vis8u, CV_8U, 255.0 / (zmax - zmin), -255.0 * zmin / (zmax - zmin));
          // cv::bitwise_not(depth_vis8u, depth_vis8u);
            //cv::applyColorMap(depth_vis8u, depth_vis8u, cv::COLORMAP_TURBO);
            //cv::Mat disp_display;
            //cv::normalize(disp_float, disp_display, 0, 255, cv::NORM_MINMAX, CV_8U);
            //cv::applyColorMap(disp_display, disp_display, cv::COLORMAP_JET); // Make it pretty

            global_depth_map = depth_map;

    // Show disparity and set mouse callback for measurement
    cv::imshow("Disparity (vis)", disp_float);
    cv::setMouseCallback("Disparity (vis)", onMouseMeasure);
            
            // cv::imshow("Rectified Left", left_rect);
            // cv::imshow("Rectified Right", right_rect);
            cv::imshow("Depth Map", depth_vis8u);
            cv::imshow("Disparity Map", disp_float);

            if (cv::waitKey(1) == 27) {
                break;
            }
        }
    }

void image_disparity_measure(const std::string &img_file) {
    StereoConfiguration config;
    if (!config.loadFromFile("config/stereo.yaml")) {
        std::cerr << "Could not load stereo.yaml" << std::endl;
        return;
    }
    StereoRectifier rectifier(config);
    StereoDisparity disparity_computer(config.Q);

    cv::Mat frame = cv::imread(img_file, cv::IMREAD_COLOR);
    if (frame.empty()) {
        std::cerr << "Could not load image: " << img_file << std::endl;
        return;
    }
    cv::Mat left_raw = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
    cv::Mat right_raw = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

    cv::Mat left_rect, right_rect;
    rectifier.rectify(left_raw, right_raw, left_rect, right_rect);

    cv::Mat disp_float = disparity_computer.computeDisparity(left_rect, right_rect);
    cv::Mat depth_map = disparity_computer.computeDepth(disp_float);

    cv::Mat depthZ;
    if (depth_map.channels() == 3) {
        cv::extractChannel(depth_map, depthZ, 2);
    } else {
        depthZ = depth_map;
    }

    // Mask valid depth values: positive, finite, and not huge
    cv::Mat z_valid = (depthZ > 0) & (depthZ < 10000) & (depthZ == depthZ);

    double zmin_raw = 0, zmax_raw = 0;
    cv::minMaxLoc(depthZ, &zmin_raw, &zmax_raw, nullptr, nullptr, z_valid);

    if (!(zmax_raw > zmin_raw)) {
        zmin_raw = 1000.0;
        zmax_raw = 2000.0;
    }

    cv::Mat depth8u;
    depthZ.convertTo(depth8u, CV_8U, 255.0 / (zmax_raw - zmin_raw), -255.0 * zmin_raw / (zmax_raw - zmin_raw));
    cv::Mat depth_vis8u;
    cv::applyColorMap(depth8u, depth_vis8u, cv::COLORMAP_TURBO);

    // Save depth map for measurement
    global_depth_map = depth_map;

    // Show and measure
    cv::imshow("Rectified Left", left_rect);
    cv::imshow("Rectified Right", right_rect);
    cv::imshow("Disparity (vis)", disp_float);
    cv::imshow("Depth Map", depth_vis8u);
    cv::setMouseCallback("Disparity (vis)", onMouseMeasure);

    std::cout << "Click two points in the Disparity (vis) window to measure distance." << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
}