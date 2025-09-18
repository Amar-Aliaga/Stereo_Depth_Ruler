#pragma once

#include <string>
#include <opencv2/opencv.hpp>

void save_frames();
void change_filename();
void capture_frame(const std::string &output_file);
void image_desparity(const std::string &img_file);
void zed_footage();
static void MouseCallbackWrapper(int event, int x, int y, int flags, void *user_data);
void image_disparity_measure(const std::string &img_file);
void specific_depth_pixel(const cv::Mat &mat);