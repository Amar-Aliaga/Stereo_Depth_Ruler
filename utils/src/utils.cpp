#include "utils.hpp"

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <cmath>     

#include <opencv2/opencv.hpp>
#include "stereo_calibrator.hpp"
#include "stereo_disparity.hpp"
#include "stereo_rectifier.hpp"
#include "stereo_configuration.hpp"
#include "pcd_write.hpp"

#include <sl/Camera.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <limits>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

static std::vector<cv::Point> clicked_points {};
static std::vector<std::pair<cv::Point, cv::Point>> points_history {};
static float dist {};
static std::vector<float> dist_vector {};

//event == cv::EVENT_LBUTTONDOWN && !(flags & cv::EVENT_FLAG_SHIFTKEY)

void onMouseMeasure(int event, int x, int y, int flags, void *user_data) {
    if (event != cv::EVENT_LBUTTONDOWN || !(flags & cv::EVENT_FLAG_SHIFTKEY)) return; 
    MouseMat *disp = static_cast<MouseMat*>(user_data);
    if(!disp || !disp->raw_map || !disp->dis_map) return;

    cv::Mat *rawMap = disp->raw_map;
    cv::Mat *disMap = disp->dis_map;

    if (x < 0 || x >= rawMap->cols || y < 0 || y >= rawMap->rows) return;
    
    clicked_points.push_back(cv::Point(x, y));

    cv::circle(*disMap, clicked_points[0], 5, cv::Scalar(0,0,255), -1);
    cv::circle(*disMap, clicked_points[1], 5, cv::Scalar(0,0,255), -1);

    if (clicked_points.size() == 2) {
        cv::Vec3f xyz1 = rawMap->at<cv::Vec3f>(clicked_points[0].y, clicked_points[0].x);
        cv::Vec3f xyz2 = rawMap->at<cv::Vec3f>(clicked_points[1].y, clicked_points[1].x);
        cv::line(*disMap, clicked_points[0], clicked_points[1], cv::Scalar(77, 211 ,187), 1);
        cv::imshow("Left: rectified image + disparity overlay", *disMap);
        dist = cv::norm(xyz1 - xyz2);
        points_history.push_back(std::make_pair(clicked_points[0], clicked_points[1]));
        dist_vector.push_back(dist);
        std::cout << "Distance: " << dist/10 << " cm" << std::endl;
        clicked_points.clear();
        //points_history.clear();
    }
}


void save_csvFile() {
    size_t i{0}, j{0};
        std::ofstream csvFile("/home/amar-aliaga/Desktop/wayland/wayland_thirdProject/results/measurements.csv");
        if(!csvFile.is_open()) {
            std::cerr << "Failed to create CSV file!" << std::endl;
            return;
        }
        csvFile << "First_point, Second_point, Distance\n";
        csvFile << std::fixed << std::setprecision(5);

        while(i < points_history.size() && j < dist_vector.size()) {
            std::cout << "First point: "  << points_history[i].first  << std::endl;
            std::cout << "Second point: " << points_history[i].second << std::endl;
            std::cout << "Distance: " << dist_vector[j] / 10 << "cm" << std::endl;

            csvFile << points_history[i].first  << ", "
                    << points_history[i].second << ", "
                    << dist_vector[j]/10 <<  " cm"  << "\n";
            i++;
            j++;
        }
        std::cout << std::endl;

        csvFile.close();

        std::cout << "Measurements saved to /results/measurements.csv" << std::endl;
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


    StereoDisparity disparity (config.Q);
        
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


// bool hasField(const pcl::PCLPointCloud2& blob, const std::string& name) {
//     for (const auto &f : blob.fields)
//         if (f.name == name) return true;
//     return false;
// }

// // Convert a depth map from OpenCV format to PCL point cloud.
// // This is a direct conversion and does not require saving to disk.
// pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertCVMatToPCL(const cv::Mat& depth_map, const cv::Mat& color_image) {
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
//     // Resize the color image to match the depth map
//     cv::Mat color_resized;
//     cv::resize(color_image, color_resized, depth_map.size());

//     for (int y = 0; y < depth_map.rows; ++y) {
//         for (int x = 0; x < depth_map.cols; ++x) {
//             // Get the 3D point and color from the image and depth map
//             cv::Vec3f point = depth_map.at<cv::Vec3f>(y, x);
//             cv::Vec3b color = color_resized.at<cv::Vec3b>(y, x);

//             // Create a PCL point
//             pcl::PointXYZRGB p;
//             p.x = point[0];
//             p.y = point[1];
//             p.z = point[2];
//             p.r = color[2];
//             p.g = color[1];
//             p.b = color[0];

//             // If the point is valid, add it to the cloud
//             if (p.z > 0 && p.z < 100000) { // filter out infinite depth
//                 cloud->points.push_back(p);
//             }
//         }
//     }
//     cloud->width = static_cast<uint32_t>(cloud->points.size());
//     cloud->height = 1;
//     return cloud;
// }


// void live_disparity_map() {
//     const std::string &s {"/home/amar-aliaga/Desktop/my_video/output.mp4"};

//     StereoConfiguration config;

//     if (!config.loadFromFile("config/stereo.yaml")) {
//         std::cerr << "Error: Could not load configuration file." << std::endl;
//         return;
//     }
//     StereoRectifier rectifier(config);
//     StereoDisparity disparity_computer(config.Q);

//     cv::VideoCapture cap(2);
//     if (!cap.isOpened()) {
//         std::cerr << "Error: Could not open video file." << std::endl;
//         return;
//     }

//     cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
//     cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

//     int w = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
//     int h = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
//     std::cout << "Res: " << w << "x" << h << std::endl;

//     // --- PCL Visualizer Setup ---
//     // -
    
//     // --- Main Processing Loop ---
//     cv::Mat frame;
//     while (true) {
//         cap >> frame;
//         // if (frame.empty()) {
//         //     std::cout << "End of video." << std::endl;
//         //     cap.set(cv::CAP_PROP_POS_FRAMES, 0); // Loop the video
//         //     continue;
//         // }

//         cv::Mat left_raw  = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
//         cv::Mat right_raw = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
        
//         cv::Mat left_rect, right_rect;
//         rectifier.rectify(left_raw, right_raw, left_rect, right_rect);
        
//         cv::Mat disp_float = disparity_computer.computeDisparity(left_rect, right_rect);
//         cv::Mat depth_map = disparity_computer.computeDepth(disp_float);
        
//         // Convert to PCL point cloud in memory
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud = convertCVMatToPCL(depth_map, left_rect);
//         pcl::io::savePCDFileBinary("/home/amar-aliaga/Desktop/wayland/wayland_thirdProject/results/frame_xxxxx.pcd", *new_cloud);
//         // // Update the point cloud in the viewer
//         //viewer.updatePointCloud<pcl::PointXYZRGB>(new_cloud, "live_cloud");

//         cv::Mat res = disparity_computer.show_depthMap(depth_map);
//         if(cv::waitKey(10) == 27) break;
//         else {
//             cv::imshow("depth", res);
//         }
//         // // Process PCL events
//         //viewer.spinOnce(10);
//     }
// }


void live_disparity_map() {
    const std::string &s {"/home/amar-aliaga/Desktop/my_video/output.mp4"};

    StereoConfiguration config;

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

    bool paused = false;
    cv::Mat stored_frame; 
    cv::Mat frozen_display_depth; 
    cv::Mat frozen_overlay;
    cv::Mat frozen_depth_map; 
    cv::Mat frozen_frozen_overlay;
    MouseMat frozen_mouse_data;

    while (true) {
        cv::Mat frame;
        cv::Mat display_depth; 
        cv::Mat overlay;       
        MouseMat mouse_data; 

        cv::Mat depth_map;
        cv::Mat clean_overlay;     // Store original overlay before drawing
        cv::Mat clean_frozen_overlay;

        if (!paused) {
            cap >> frame;
            if (frame.empty()) {
                std::cout << "End of video." << std::endl;
                break;
            }
            stored_frame = frame.clone();

            cv::Mat left_raw = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
            cv::Mat right_raw = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

            cv::Mat left_rect, right_rect;
            rectifier.rectify(left_raw, right_raw, left_rect, right_rect);
            
            cv::Mat disp_float = disparity_computer.computeDisparity(left_rect, right_rect);
            cv::Mat depth_map = disparity_computer.computeDepth(disp_float);

            display_depth = disparity_computer.show_depthMap(depth_map);
            cv::Mat display_disparity = disparity_computer.show_disparityMap(disp_float);

            cv::Mat disparity_heatmap;
            cv::applyColorMap(display_disparity, disparity_heatmap, cv::COLORMAP_JET);

            cv::resize(disparity_heatmap, disparity_heatmap, display_depth.size());
            cv::resize(left_rect, left_rect, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

            cv::addWeighted(left_rect, 0.7, disparity_heatmap, 0.3, 0, overlay);

            mouse_data.raw_map = &depth_map;
            mouse_data.dis_map = &overlay;

            frozen_display_depth = display_depth.clone();  
            frozen_overlay = overlay.clone();           
            frozen_depth_map = depth_map.clone(); 

            clean_overlay = overlay.clone();
            clean_frozen_overlay = frozen_overlay.clone();
            
            frozen_mouse_data.raw_map = &frozen_depth_map;
            frozen_mouse_data.dis_map = &frozen_overlay;
        }

        cv::imshow("Depth Map", paused ? frozen_display_depth : display_depth);
        cv::imshow("Left: rectified image + disparity overlay", paused ? frozen_overlay : overlay);
        
        cv::setMouseCallback("Left: rectified image + disparity overlay", 
                            onMouseMeasure, 
                            paused ? &frozen_mouse_data : &mouse_data);

        int key = cv::waitKey(paused ? 100 : 1);

        if(key == 115 || key == 83) {
            save_csvFile();
        }
        
        if (key == 27) {
            break;
        } else if (key == 102 || key == 70) {
            paused = !paused;
            std::cout << (paused ? "Video paused" : "Video resumed") << std::endl;
        } else if (key == 114 || key == 82) {
            clicked_points.clear(); 
            points_history.clear();
            dist_vector.clear();
            if(clicked_points.empty() && points_history.empty() && dist_vector.empty()) {
                std::cout << "Values have been reseted." << std::endl;
            }else {
                std::cout << "Values have NOT been reseted." << std::endl;
            }
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
    cv::Mat left_raw  = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
    cv::Mat right_raw = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

    cv::Mat left_rect, right_rect;
    rectifier.rectify(left_raw, right_raw, left_rect, right_rect);

    cv::Mat disp_float = disparity_computer.computeDisparity(left_rect, right_rect);
    cv::Mat depth_map = disparity_computer.computeDepth(disp_float);

    cv::Mat show_disp = disparity_computer.show_disparityMap(disp_float);
    cv::Mat show_depth = disparity_computer.show_depthMap(depth_map);

    MouseMat mouse_data;
    mouse_data.raw_map = &disp_float;
    mouse_data.dis_map = &show_disp;

    cv::imshow("Rectified Left", left_rect);
    cv::imshow("Rectified Right", right_rect);
    cv::imshow("Disparity (vis)", show_disp);
    cv::imshow("Depth Map", show_depth);
    cv::setMouseCallback("Disparity (vis)", onMouseMeasure, &mouse_data);

    std::cout << "Click two points in the Disparity (vis) window to measure distance." << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
}


void display_pointCloud() { 
    // StereoConfiguration config;

    // if (!config.loadFromFile("config/stereo.yaml")) {
    //     return;
    // }
    // StereoRectifier rectifier(config);

    // StereoDisparity disparity_computer(config.Q);

    // cv::Mat disp_float = disparity_computer.computeDisparity(left_rect, right_rect);
    // cv::Mat depth_map = disparity_computer.computeDepth(disp_float);

    // PointCloudT::Ptr cloud2 = convertCVMatToPCL(depth_map);
    // pcl::io::savePCDFileBinary("results/frame_xxxxx.pcd", *cloud2);
    std::cout << "Hello" << std::endl;
}