cmake_minimum_required(VERSION 2.8.12)

project(rtmpClient)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#cmake设置动态库路径,也可使用gcc指定
set(CMAKE_SKIP_BUILD_RPATH FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
SET(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/lib:$ORIGIN/../lib:$ORIGIN/../")   #指定运行时动态库的加载路径,ORIGIN指运行文件所在目录

#resolve the keyword
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -pthread")

#normal setting
include_directories(
    /opt/nvidia/deepstream/deepstream/sources/includes
    /opt/nvidia/deepstream/deepstream/sources/apps/apps-common/includes
    /usr/include/glib-2.0
    /usr/lib/aarch64-linux-gnu/glib-2.0/include
    /usr/include/gstreamer-1.0
    /usr/local/include
    /usr/local/include/gstreamer-1.0
    /usr/local/cuda/include
    /usr/local/cuda/targets/aarch64-linux/include
    /usr/include/opencv4
)

link_directories(
    /usr/lib/aarch64-linux-gnu
    /usr/lib/aarch64-linux-gnu/gstreamer-1.0
    /opt/nvidia/deepstream/deepstream/lib/
    /opt/nvidia/deepstream/deepstream/lib/gst-plugins/
    /usr/local/cuda/lib64
    /usr/local/cuda/targets/aarch64-linux/lib
    /usr/local/lib
)

find_package(Qt5 COMPONENTS Core Network Widgets REQUIRED)

add_executable(${PROJECT_NAME}
    main.cpp
    rtmpstream.h
    rtmpstream.cpp
    global.h
    global.cpp
    tools.h
)

target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Network Qt5::Widgets)
target_link_libraries(${PROJECT_NAME} cudart nvdsgst_helper nvdsgst_meta nvds_meta nvbufsurface nvbufsurftransform nvds_utils nvdsgst_inferserver nvdsgst_udp gstnvvideo4linux2)
target_link_libraries(${PROJECT_NAME} gstrtspserver-1.0 gstreamer-1.0 glib-2.0 gobject-2.0 gstapp-1.0)
target_link_libraries(${PROJECT_NAME} opencv_core opencv_imgproc opencv_highgui opencv_videoio opencv_imgcodecs)


