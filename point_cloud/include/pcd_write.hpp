#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/common/common.h> 


class PointCloud {
    private:
        using PointT = pcl::PointXYZRGB;
        using PointCloudT = pcl::PointCloud<PointT>;
    public:
        PointCloud() = default;
        PointCloudT::Ptr convertCVMatToPCL(const cv::Mat& pointCloud_CV, const cv::Mat& colorImage_CV = cv::Mat());
        void save_and_display_pointcloud();
};
