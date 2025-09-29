#include "pcd_write.hpp"
#include "stereo_configuration.hpp"
#include "stereo_disparity.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <limits>
#include <string>
#include <iomanip> 


PointCloud::PointCloudT::Ptr PointCloud::convertCVMatToPCL(const cv::Mat& pointCloud_CV, const cv::Mat& colorImage_CV) {
    PointCloudT::Ptr cloud(new PointCloudT);

    if (pointCloud_CV.empty() || pointCloud_CV.type() != CV_32FC3) {
        std::cerr << "ERROR: Input point cloud is empty or not of type CV_32FC3." << std::endl;
        return cloud;
    }

    bool hasColor = !colorImage_CV.empty() && colorImage_CV.size() == pointCloud_CV.size();
    cloud->width = pointCloud_CV.cols;
    cloud->height = pointCloud_CV.rows;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    for (int v = 0; v < pointCloud_CV.rows; ++v) {
        for (int u = 0; u < pointCloud_CV.cols; ++u) {
            cv::Vec3f point = pointCloud_CV.at<cv::Vec3f>(v, u);
            size_t index = v * pointCloud_CV.cols + u;
            if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
                cloud->points[index].x = point[0];
                cloud->points[index].y = point[1];
                cloud->points[index].z = point[2];
                if (hasColor) {
                    cv::Vec3b bgr = colorImage_CV.at<cv::Vec3b>(v, u);
                    cloud->points[index].r = bgr[2];
                    cloud->points[index].g = bgr[1];
                    cloud->points[index].b = bgr[0];
                }
            } else {
                cloud->points[index].x = cloud->points[index].y = cloud->points[index].z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    return cloud;
}


void PointCloud::show_pointCloud(const cv::Mat &left, const cv::Mat &pointCloud_CV, const float voxel_size) {
    if (left.empty() || left.cols % 2 != 0) {
        std::cerr << "Invalid stereo image!" << std::endl;
        return;
    }

    PointCloudT::Ptr pcl_cloud = convertCVMatToPCL(pointCloud_CV, left);

    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(pcl_cloud);
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);

    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    voxel_filter.filter(*cloud_filtered);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCD Viewer"));
    viewer->setSize(640, 360); 
    viewer->getRenderWindow()->SetPosition(3100, 300); 
    viewer->addPointCloud<PointT>(cloud_filtered, "cloud");
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}


void PointCloud::show_live_pointCloud() {
    const std::string video_path1 = "/home/amar-aliaga/Downloads/cam.mp4";
    const std::string video_path2 = "assets/output.mp4";
    const std::string Q_matrix_path = "/home/amar-aliaga/Desktop/wayland/wayland_thirdProject/config/stereo.yaml";
    const std::string output_dir = "/home/amar-aliaga/Desktop/wayland/wayland_thirdProject/results/";
    const int target_frame = 100;

    cv::VideoCapture cap(video_path2);
    if (!cap.isOpened()) {
        std::cerr << "Could not open video: " << video_path1 << std::endl;
        return;
    }

    cv::Mat frame;
    for (int i = 0; i <= target_frame; ++i) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Could not read frame " << i << std::endl;
            return;
        }
    }

    if (frame.cols % 2 != 0) {
        std::cerr << "Frame width is not even; cannot split into left/right images." << std::endl;
        return;
    }

    int width = frame.cols / 2;
    cv::Mat left  = frame(cv::Rect(0, 0, width, frame.rows)).clone();
    cv::Mat right = frame(cv::Rect(width, 0, width, frame.rows)).clone();

    cv::Mat left_gray, right_gray;
    cv::cvtColor(left, left_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right, right_gray, cv::COLOR_BGR2GRAY);

    cv::FileStorage fs(Q_matrix_path, cv::FileStorage::READ);
    cv::Mat Q;
    fs["Q"] >> Q;
    fs.release();
    if (Q.empty()) {
        std::cerr << "Could not load Q matrix from " << Q_matrix_path << std::endl;
        return;
    }

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 80, 5,
        8 * 5 * 5 * 3,
        32 * 5 * 5 * 3,
        1, 63, 12, 200, 2,
        cv::StereoSGBM::MODE_SGBM_3WAY
    );

    cv::Mat disp, disp_float;
    sgbm->compute(left_gray, right_gray, disp);
    disp.convertTo(disp_float, CV_32F, 1.0 / 16.0);

    // Reproject to 3D
    cv::Mat pointCloud_CV;
    cv::reprojectImageTo3D(disp_float, pointCloud_CV, Q, true);


    PointCloudT::Ptr pcl_cloud = convertCVMatToPCL(pointCloud_CV, left);
    std::cout << "Original points: " << pcl_cloud->size() << std::endl;

    float voxel_size = 0.005f;
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(pcl_cloud);
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);

    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    voxel_filter.filter(*cloud_filtered);
    cloud_filtered->is_dense = false;

    std::cout << "Filtered points: " << cloud_filtered->size() << std::endl;

    std::stringstream ss;
    ss << output_dir << "frame_" << std::setfill('0') << std::setw(5) << target_frame << ".pcd";
    std::string out_path = ss.str();

    if (cloud_filtered->points.size() > 0) {
        pcl::io::savePCDFileBinary(out_path, *cloud_filtered);
        std::cout << "Saved downsampled point cloud to " << out_path << std::endl;
    } else {
        std::cerr << "No points to save after downsampling!" << std::endl;
        return;
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCD Viewer"));
    viewer->setSize(800, 600); 
    viewer->addPointCloud<PointT>(cloud_filtered, "cloud");
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}
