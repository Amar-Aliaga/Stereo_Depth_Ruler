#pragma once

#include <string>

void change_filename();
void save_frames();
void capture_frame(const std::string &output_file);
void image_desparity(const std::string &img_file);
void zed_footage();
void live_disparity_map();