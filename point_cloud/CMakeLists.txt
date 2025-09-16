project(point_cloud)

find_package(OpenCV REQUIRED)
find_package(PCL 1.14 REQUIRED COMPONENTS common io)

add_executable(point_cloud src/pcd_write.cpp)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_include_directories(point_cloud PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(point_cloud ${OpenCV_LIBS} ${PCL_LIBRARIES})
