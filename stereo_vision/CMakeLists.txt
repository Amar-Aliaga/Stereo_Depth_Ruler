file(GLOB SRC_FILES src/*.cpp)
file(GLOB HEADER_FILES include/*.hpp)

add_library(stereo_vision ${SRC_FILES} ${HEADER_FILES})

find_package(OpenCV REQUIRED)
target_include_directories(stereo_vision PUBLIC include)
target_link_libraries(stereo_vision PUBLIC ${OpenCV_LIBS})

