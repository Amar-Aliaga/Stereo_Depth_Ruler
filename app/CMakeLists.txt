find_package(OpenCV REQUIRED)

add_executable(stereo_ruler stereo_ruler.cpp)

target_include_directories(stereo_ruler PRIVATE
    ${CMAKE_SOURCE_DIR}/stereo_vision/include
    ${CMAKE_SOURCE_DIR}/utils/include
    ${CMAKE_SOURCE_DIR}/point_cloud/include
)

target_link_libraries(stereo_ruler PRIVATE
    utils
    stereo_vision
    point_cloud
    ${OpenCV_LIBS}
)
