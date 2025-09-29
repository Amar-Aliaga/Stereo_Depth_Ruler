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

void PointCloud::show_pointcloud(const cv::Mat &left, const cv::Mat &pointCloud_CV, const float voxel_size) {
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
