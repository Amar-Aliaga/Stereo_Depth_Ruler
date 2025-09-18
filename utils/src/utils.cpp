// #include "utils.hpp"

// #include <opencv2/opencv.hpp>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/io/pcd_io.h> 
// #include <pcl/common/io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/PCLPointCloud2.h>
// #include <pcl/conversions.h>
// #include <pcl/filters/voxel_grid.h>
// #include <cmath>     

// #include "stereo_calibrator.hpp"
// #include "stereo_disparity.hpp"
// #include "stereo_rectifier.hpp"
// #include "stereo_configuration.hpp"
// #include "pcd_write.hpp"

// #include <sl/Camera.hpp>
// #include <iostream>
// #include <vector>
// #include <string>
// #include <filesystem>
// #include <limits>
// #include <iomanip>


// typedef pcl::PointXYZRGB PointT;
// typedef pcl::PointCloud<PointT> PointCloudT;


// // bool hasField(const pcl::PCLPointCloud2& blob, const std::string& name) {
// //     for (const auto &f : blob.fields)
// //         if (f.name == name) return true;
// //     return false;
// // }

// // // Convert a depth map from OpenCV format to PCL point cloud.
// // // This is a direct conversion and does not require saving to disk.
// // pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertCVMatToPCL(const cv::Mat& depth_map, const cv::Mat& color_image) {
// //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
// //     // Resize the color image to match the depth map
// //     cv::Mat color_resized;
// //     cv::resize(color_image, color_resized, depth_map.size());

// //     for (int y = 0; y < depth_map.rows; ++y) {
// //         for (int x = 0; x < depth_map.cols; ++x) {
// //             // Get the 3D point and color from the image and depth map
// //             cv::Vec3f point = depth_map.at<cv::Vec3f>(y, x);
// //             cv::Vec3b color = color_resized.at<cv::Vec3b>(y, x);

// //             // Create a PCL point
// //             pcl::PointXYZRGB p;
// //             p.x = point[0];
// //             p.y = point[1];
// //             p.z = point[2];
// //             p.r = color[2];
// //             p.g = color[1];
// //             p.b = color[0];

// //             // If the point is valid, add it to the cloud
// //             if (p.z > 0 && p.z < 100000) { // filter out infinite depth
// //                 cloud->points.push_back(p);
// //             }
// //         }
// //     }
// //     cloud->width = static_cast<uint32_t>(cloud->points.size());
// //     cloud->height = 1;
// //     return cloud;
// // }


// // void live_disparity_map() {
// //     const std::string &s {"/home/amar-aliaga/Desktop/my_video/output.mp4"};

// //     StereoConfiguration config;

// //     if (!config.loadFromFile("config/stereo.yaml")) {
// //         std::cerr << "Error: Could not load configuration file." << std::endl;
// //         return;
// //     }
// //     StereoRectifier rectifier(config);
// //     StereoDisparity disparity_computer(config.Q);

// //     cv::VideoCapture cap(2);
// //     if (!cap.isOpened()) {
// //         std::cerr << "Error: Could not open video file." << std::endl;
// //         return;
// //     }

// //     cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
// //     cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

// //     int w = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
// //     int h = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
// //     std::cout << "Res: " << w << "x" << h << std::endl;

// //     // --- PCL Visualizer Setup ---
// //     // -
    
// //     // --- Main Processing Loop ---
// //     cv::Mat frame;
// //     while (true) {
// //         cap >> frame;
// //         // if (frame.empty()) {
// //         //     std::cout << "End of video." << std::endl;
// //         //     cap.set(cv::CAP_PROP_POS_FRAMES, 0); // Loop the video
// //         //     continue;
// //         // }

// //         cv::Mat left_raw  = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
// //         cv::Mat right_raw = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
        
// //         cv::Mat left_rect, right_rect;
// //         rectifier.rectify(left_raw, right_raw, left_rect, right_rect);
        
// //         cv::Mat disp_float = disparity_computer.computeDisparity(left_rect, right_rect);
// //         cv::Mat depth_map = disparity_computer.computeDepth(disp_float);
        
// //         // Convert to PCL point cloud in memory
// //         pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud = convertCVMatToPCL(depth_map, left_rect);
// //         pcl::io::savePCDFileBinary("/home/amar-aliaga/Desktop/wayland/wayland_thirdProject/results/frame_xxxxx.pcd", *new_cloud);
// //         // // Update the point cloud in the viewer
// //         //viewer.updatePointCloud<pcl::PointXYZRGB>(new_cloud, "live_cloud");

// //         cv::Mat res = disparity_computer.show_depthMap(depth_map);
// //         if(cv::waitKey(10) == 27) break;
// //         else {
// //             cv::imshow("depth", res);
// //         }
// //         // // Process PCL events
// //         //viewer.spinOnce(10);
// //     }
// // }


// // void live_disparity_map() {
// //     const std::string &s {"/home/amar-aliaga/Desktop/my_video/output.mp4"};
// //     const std::string &v {"/home/amar-aliaga/Downloads/cam.mp4"};

// //     StereoConfiguration config;

// //     if (!config.loadFromFile("config/stereo.yaml")) {
// //         std::cerr << "Error: Could not load configuration file." << std::endl;
// //         return;
// //     }
// //     StereoRectifier rectifier(config);
// //     StereoDisparity disparity_computer(config.Q);

// //     cv::VideoCapture cap(2);
// //     if (!cap.isOpened()) {
// //         std::cerr << "Error: Could not open video file." << std::endl;
// //         return;
// //     }

// //     cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
// //     cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

// //     int w = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
// //     int h = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
// //     std::cout << "Res: " << w << "x" << h << std::endl;

// //     bool paused = false;
// //     cv::Mat stored_frame; 
// //     cv::Mat frozen_display_depth; 
// //     cv::Mat frozen_overlay;
// //     cv::Mat frozen_depth_map; 
// //     cv::Mat frozen_frozen_overlay;
// //     MouseMat frozen_mouse_data;

// //     while (true) {
// //         cv::Mat frame;
// //         cv::Mat display_depth; 
// //         cv::Mat overlay;       
// //         MouseMat mouse_data; 

// //         cv::Mat depth_map;
// //         cv::Mat clean_overlay;     
// //         cv::Mat clean_frozen_overlay;

// //         if (!paused) {
// //             cap >> frame;
// //             if (frame.empty()) {
// //                 std::cout << "End of video." << std::endl;
// //                 break;
// //             }
// //             stored_frame = frame.clone();

// //             cv::Mat left_raw = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
// //             cv::Mat right_raw = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

// //             cv::Mat left_rect, right_rect;
// //             rectifier.rectify(left_raw, right_raw, left_rect, right_rect);
            
// //             cv::Mat disp_float = disparity_computer.computeDisparity(left_rect, right_rect);
// //             cv::Mat depth_map = disparity_computer.computeDepth(disp_float);

// //             display_depth = disparity_computer.show_depthMap(depth_map);
// //             cv::Mat display_disparity = disparity_computer.show_disparityMap(disp_float);

// //             cv::Mat disparity_heatmap;
// //             cv::applyColorMap(display_disparity, disparity_heatmap, cv::COLORMAP_JET);

// //             cv::resize(disparity_heatmap, disparity_heatmap, display_depth.size());
// //             cv::resize(left_rect, left_rect, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

// //             cv::addWeighted(left_rect, 0.7, disparity_heatmap, 0.3, 0, overlay);

// //             mouse_data.raw_map = &depth_map;
// //             mouse_data.dis_map = &overlay;

// //             frozen_display_depth = display_depth.clone();  
// //             frozen_overlay = overlay.clone();           
// //             frozen_depth_map = depth_map.clone(); 

// //             clean_overlay = overlay.clone();
// //             clean_frozen_overlay = frozen_overlay.clone();
            
// //             frozen_mouse_data.raw_map = &frozen_depth_map;
// //             frozen_mouse_data.dis_map = &frozen_overlay;
// //         }

// //         cv::imshow("Depth Map", paused ? frozen_display_depth : display_depth);
// //         cv::imshow("Left: rectified image + disparity overlay", paused ? frozen_overlay : overlay);
        
// //         cv::setMouseCallback("Left: rectified image + disparity overlay", 
// //                             onMouseMeasure, 
// //                             paused ? &frozen_mouse_data : &mouse_data);

// //         depth_coverage(frozen_depth_map);

// //         int key = cv::waitKey(paused ? 100 : 1);

// //         if(key == 115 || key == 83) {
// //             save_csvFile();
// //         }
        
// //         if (key == 27) {
// //             break;
// //         } else if (key == 102 || key == 70) {
// //             paused = !paused;
// //             std::cout << (paused ? "Video paused" : "Video resumed") << std::endl;
// //         } else if (key == 114 || key == 82) {
// //             clicked_points.clear(); 
// //             points_history.clear();
// //             dist_vector.clear();
// //             if(clicked_points.empty() && points_history.empty() && dist_vector.empty()) {
// //                 std::cout << "Values have been reseted." << std::endl;
// //             }else {
// //                 std::cout << "Values have NOT been reseted." << std::endl;
// //             }
// //         }
// //     }
// // }


// void display_pointCloud() { 
//     // StereoConfiguration config;

//     // if (!config.loadFromFile("config/stereo.yaml")) {
//     //     return;
//     // }
//     // StereoRectifier rectifier(config);

//     // StereoDisparity disparity_computer(config.Q);

//     // cv::Mat disp_float = disparity_computer.computeDisparity(left_rect, right_rect);
//     // cv::Mat depth_map = disparity_computer.computeDepth(disp_float);

//     // PointCloudT::Ptr cloud2 = convertCVMatToPCL(depth_map);
//     // pcl::io::savePCDFileBinary("results/frame_xxxxx.pcd", *cloud2);
//     std::cout << "Hello" << std::endl;
// }