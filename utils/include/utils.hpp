#pragma once


#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <string>

struct MouseMat {
    cv::Mat *raw_map = nullptr;
    cv::Mat *dis_map = nullptr;
};

void onMouseMeasure(int event, int x, int y, int, void*);
void save_csvFile();

bool hasField(const pcl::PCLPointCloud2& blob, const std::string& name);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertCVMatToPCL(const cv::Mat& depth_map, const cv::Mat& color_image);

void live_disparity_map();
void display_pointCloud();
