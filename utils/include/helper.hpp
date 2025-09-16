#pragma once

#include <string>

void save_frames();
void change_filename();
void capture_frame(const std::string &output_file);
void image_desparity(const std::string &img_file);
void zed_footage();
void image_disparity_measure(const std::string &img_file);