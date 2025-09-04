cmake_minimum_required(VERSION 3.16)

find_package(OpenCV REQUIRED)

file(GLOB SRC_FILES src/*.cpp)
file(GLOB HEADER_FILES include/*.hpp)

add_library(utils ${SRC_FILES} ${HEADER_FILES})

target_include_directories(utils PUBLIC include ${OpenCV_INCLUDE_DIRS})

target_link_libraries(utils PUBLIC ${OpenCV_LIBS} stereo_vision)