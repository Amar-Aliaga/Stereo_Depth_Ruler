#include "pcd_write.hpp"
#include "stereo_displayer.hpp"
#include "stereo_calibrator.hpp"
#include "stereo_disparity.hpp"
#include "stereo_rectifier.hpp"
#include "stereo_configuration.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/common/io.h>   
#include <pcl/visualization/cloud_viewer.h>

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
#include <limits>



typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/**
 * Converts an OpenCV 3D Point Cloud Mat and an optional color Mat to a PCL PointCloud.
 * @param pointCloud_CV Input OpenCV Mat (CV_32FC3) from reprojectImageTo3D
 * @param colorImage_CV Input OpenCV Mat (CV_8UC3) for color, must be same size as pointCloud_CV. If empty, no color is added.
 * @return A shared pointer to the created PCL PointCloud.
 */


PointCloudT::Ptr convertCVMatToPCL(const cv::Mat& pointCloud_CV, const cv::Mat& colorImage_CV) {
    // Create the PCL point cloud object
    PointCloudT::Ptr cloud(new PointCloudT);

    // Check if the input point cloud is valid and organized
    if (pointCloud_CV.empty() || pointCloud_CV.type() != CV_32FC3) {
        std::cerr << "ERROR: Input point cloud is empty or not of type CV_32FC3." << std::endl;
        return cloud;
    }

    bool hasColor = !colorImage_CV.empty() && colorImage_CV.size() == pointCloud_CV.size();

    // Set the PCL cloud to be organized (same width and height as the image)
    cloud->width = pointCloud_CV.cols;
    cloud->height = pointCloud_CV.rows;
    cloud->is_dense = false; // Because it will have NaN/inf points we need to filter
    cloud->points.resize(cloud->width * cloud->height);

    // Iterate through every pixel in the point cloud
    for (int v = 0; v < pointCloud_CV.rows; ++v) {       // y coordinate
        for (int u = 0; u < pointCloud_CV.cols; ++u) {   // x coordinate

            // Get the 3D point from the OpenCV Mat
            cv::Vec3f point = pointCloud_CV.at<cv::Vec3f>(v, u);
            // Calculate the index for the PCL point cloud
            size_t index = v * pointCloud_CV.cols + u;

            // Check if the point is valid (finite)
            if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
                // Set the coordinates
                cloud->points[index].x = point[0];
                cloud->points[index].y = point[1];
                cloud->points[index].z = point[2];

                // Set the color if available
                if (hasColor) {
                    cv::Vec3b bgr = colorImage_CV.at<cv::Vec3b>(v, u);
                    // PCL uses RGB stored in a uint32_t
                    cloud->points[index].r = bgr[2]; // R from OpenCV's BGR
                    cloud->points[index].g = bgr[1]; // G
                    cloud->points[index].b = bgr[0]; // B
                }
            } else {
                // Set invalid points to NaN. PCL will ignore them if is_dense is false.
                cloud->points[index].x = cloud->points[index].y = cloud->points[index].z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    return cloud;
}


void save_as_binary() {
    StereoConfiguration config;
    std::string video_path = "/home/amar-aliaga/Desktop/my_video/output.mp4";
    int target_frame = 100; 

    // Open video
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "Could not open video: " << video_path << std::endl;
        return;
    }

    // Grab left and right frames (for demonstration, we use consecutive frames)
    cv::Mat left, right;
    for (int i = 0; i <= target_frame; ++i) {
        cap >> left;
        if (left.empty()) {
            std::cerr << "Could not read frame " << i << std::endl;
            return;
        }
    }
    cap >> right;
    if (right.empty()) {
        std::cerr << "Could not read right frame." << std::endl;
        return;
    }

    // Convert to grayscale if needed
    cv::Mat left_gray, right_gray;
    cv::cvtColor(left, left_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right, right_gray, cv::COLOR_BGR2GRAY);

    // Load Q matrix (from calibration)
    cv::FileStorage fs("/home/amar-aliaga/Desktop/wayland/wayland_thirdProject/config/stereo.yaml", cv::FileStorage::READ);
    cv::Mat Q;
    fs["Q"] >> Q;
    fs.release();
    if (Q.empty()) {
        std::cerr << "Could not load Q matrix from config/stereo.yaml" << std::endl;
        return;
    }

    // Compute disparity (replace with your real stereo pipeline!)
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 128, 5);
    cv::Mat disp, disp_float;
    sgbm->compute(left_gray, right_gray, disp);
    disp.convertTo(disp_float, CV_32F, 1.0 / 16.0);

    // Reproject to 3D
    cv::Mat pointCloud_CV;
    cv::reprojectImageTo3D(disp_float, pointCloud_CV, Q, true);

    // Convert to PCL and save
    PointCloudT::Ptr pcl_cloud = convertCVMatToPCL(pointCloud_CV, left); // Use left image for color

    if (pcl_cloud->points.size() > 0) {
        pcl::io::savePCDFileBinary("/home/amar-aliaga/Desktop/wayland/wayland_thirdProject/results/frame_xxxxx.pcd", *pcl_cloud);
        std::cout << "Saved point cloud to results/frame_xxxxx.pcd" << std::endl;
    } else {
        std::cerr << "No points to save!" << std::endl;
        return;
    }
}



void view_pcd(const std::string& filename) {
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(filename, *cloud) == -1) {
        std::cerr << "Couldn't read file " << filename << std::endl;
        return;
    }
    std::cout << "Loaded " << cloud->points.size() << " points from " << filename << std::endl;

    pcl::visualization::CloudViewer viewer("PCD Viewer");
    viewer.showCloud(cloud);

    while (!viewer.wasStopped()) {
    }
}


int main() {
    save_as_binary();

    // Now display the saved PCD file
    view_pcd("/home/amar-aliaga/Desktop/wayland/wayland_thirdProject/results/frame_xxxxx.pcd");

    return 0;
}
