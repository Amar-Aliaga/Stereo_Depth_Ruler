project(point_cloud)

find_package(OpenCV REQUIRED)
find_package(PCL 1.14 REQUIRED COMPONENTS common io visualization filters)
find_package(OpenCV REQUIRED ximgproc)

set(SOURCE  src/pcd_write.cpp)
set(INCLUDE include/pcd_write.hpp)

add_library(point_cloud STATIC ${SOURCE} ${INCLUDE})

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions (${PCL_DEFINITIONS})

target_include_directories(point_cloud PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    ../../stereo_vision/include)

target_link_libraries(point_cloud ${OpenCV_LIBS} ${PCL_LIBRARIES} stereo_vision utils)
