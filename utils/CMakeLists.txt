find_package(OpenCV REQUIRED)

set(SRC_FILES
    src/utils.cpp
)

set(HEADER_FILES
    include/utils.hpp
)

add_library(utils ${SRC_FILES} ${HEADER_FILES})

target_include_directories(utils PUBLIC include ${OpenCV_INCLUDE_DIRS})

target_link_libraries(utils PUBLIC ${OpenCV_LIBS} stereo_vision)
