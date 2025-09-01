cmake_minimum_required(VERSION 3.16)

find_package(OpenCV REQUIRED)
find_package(ZED REQUIRED)

file(GLOB SRC_FILES src/*.cpp)
file(GLOB HEADER_FILES include/*.hpp)

add_library(stereo_vision ${SRC_FILES} ${HEADER_FILES})

target_include_directories(stereo_vision PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
)

if (TARGET sl_zed::zed)
  target_link_libraries(stereo_vision PUBLIC sl_zed::zed ${OpenCV_LIBS})
elseif (TARGET ZED::ZED)
  target_link_libraries(stereo_vision PUBLIC ZED::ZED ${OpenCV_LIBS})
elseif (TARGET ZED::ZED_SDK)
  target_link_libraries(stereo_vision PUBLIC ZED::ZED_SDK ${OpenCV_LIBS})
elseif (DEFINED ZED_LIBRARIES)
  target_link_libraries(stereo_vision PUBLIC ${ZED_LIBRARIES} ${OpenCV_LIBS})
else()
  message(FATAL_ERROR "ZED SDK found but no usable target/vars (sl_zed::zed / ZED::ZED / ZED::ZED_SDK / ZED_LIBRARIES).")
endif()

set_target_properties(stereo_vision PROPERTIES
  BUILD_WITH_INSTALL_RPATH ON
  INSTALL_RPATH "/usr/local/zed/lib"
)