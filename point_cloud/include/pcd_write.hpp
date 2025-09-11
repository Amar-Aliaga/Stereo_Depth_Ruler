#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/common/common.h> 

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


PointCloudT::Ptr convertCVMatToPCL(const cv::Mat& pointCloud_CV, const cv::Mat& colorImage_CV = cv::Mat());