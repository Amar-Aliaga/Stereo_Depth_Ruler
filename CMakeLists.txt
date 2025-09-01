cmake_minimum_required(VERSION 3.16)
project(StereoVisionProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (POLICY CMP0146)
  cmake_policy(SET CMP0146 OLD)  
endif()
if (POLICY CMP0153)
  cmake_policy(SET CMP0153 OLD) 
endif()

list(APPEND CMAKE_PREFIX_PATH "/usr/local/zed")

add_subdirectory(stereo_vision)
add_subdirectory(utils)
add_subdirectory(app)