cmake_minimum_required(VERSION 3.10)
project(utils)


find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)

set(SRC_FILES
    src/utils.cpp
    src/helper.cpp
)

set(HEADER_FILES
    include/utils.hpp
    include/helper.hpp
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories   (${PCL_LIBRARY_DIRS})
add_definitions    (${PCL_DEFINITIONS})

add_library(utils STATIC ${SRC_FILES} ${HEADER_FILES})

target_include_directories(utils PUBLIC ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(utils PUBLIC ${PCL_LIBRARIES} ${OpenCV_LIBS} stereo_vision)
