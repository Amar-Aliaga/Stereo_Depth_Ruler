cmake_minimum_required(VERSION 3.10)
project(StereoVisionProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_subdirectory(app)
add_subdirectory(stereo_vision)
add_subdirectory(utils)

