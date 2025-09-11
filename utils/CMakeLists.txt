find_package(OpenCV REQUIRED)

# set(SRC_FILES
#     src/utils.cpp
# )

set(HEADER_FILES
    include/utils.hpp
)

# add_library(utils ${SRC_FILES} ${HEADER_FILES})

# target_include_directories(utils PUBLIC include ${OpenCV_INCLUDE_DIRS})

# target_link_libraries(utils PUBLIC ${OpenCV_LIBS} ${PCL_LIBRARIES} stereo_vision point_cloud)

cmake_minimum_required(VERSION 3.10)
project(utils)

find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(utils STATIC src/utils.cpp ${HEADER_FILES})

target_include_directories(utils PUBLIC ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ensure PCL visualization libs are linked into utils
target_link_libraries(utils PUBLIC ${PCL_LIBRARIES} ${OpenCV_LIBS} stereo_vision point_cloud)