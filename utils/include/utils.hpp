#pragma once


#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <string>


void onMouseMeasure(int event, int x, int y, int, void*);
void change_filename();
void save_frames();
void capture_frame(const std::string &output_file);
void image_desparity(const std::string &img_file);
void zed_footage();
bool hasField(const pcl::PCLPointCloud2& blob, const std::string& name);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertCVMatToPCL(const cv::Mat& depth_map, const cv::Mat& color_image);
void live_disparity_map();
void image_disparity_measure(const std::string &img_file);
void display_pointCloud();
