# project(point_cloud)

# find_package(PCL 1.14 REQUIRED)
# find_package(OpenCV REQUIRED)   

# add_library(point_cloud STATIC
#     src/point_cloud.cpp
# )

# target_include_directories(point_cloud PRIVATE
#     ${CMAKE_SOURCE_DIR}/stereo_vision/include
#     ${CMAKE_SOURCE_DIR}/utils/include
#     ${OpenCV_INCLUDE_DIRS}
#     ${PCL_INCLUDE_DIRS}
# )


# #include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})

# add_executable(${PROJECT_NAME} pcd_write.cpp)
# target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} ${PCL_LIBRARIES})


project(point_cloud)

find_package(OpenCV REQUIRED)
find_package(PCL 1.14 REQUIRED COMPONENTS common io)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
#include_directories(/usr/include/vtk)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(point_cloud STATIC
    src/pcd_write.cpp
)

target_include_directories(point_cloud PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(point_cloud PUBLIC ${OpenCV_LIBS} ${PCL_LIBRARIES})
