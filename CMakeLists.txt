cmake_minimum_required(VERSION 2.8.12)

project(vehicleAnalysis)

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

# output path
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# Debug
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

# Release
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

#resolve the keyword
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -pthread")

#PSDK related
execute_process(COMMAND uname -m OUTPUT_VARIABLE DEVICE_SYSTEM_ID)
if (DEVICE_SYSTEM_ID MATCHES x86_64)
    set(LIBRARY_PATH ${PROJECT_SOURCE_DIR}/psdk/psdk_lib/lib/x86_64-linux-gnu-gcc)
elseif (DEVICE_SYSTEM_ID MATCHES aarch64)
    set(LIBRARY_PATH ${PROJECT_SOURCE_DIR}/psdk/psdk_lib/lib/aarch64-linux-gnu-gcc)
else ()
    message(FATAL_ERROR "FATAL: Please confirm your platform.")
endif ()

install(FILES ${LIBRARY_PATH}/libpayloadsdk.a
        DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
        )

install(DIRECTORY psdk_lib/include
        DESTINATION "${CMAKE_INSTALL_PREFIX}"
        )

if (NOT USE_SYSTEM_ARCH)
    add_definitions(-DSYSTEM_ARCH_LINUX)
endif ()

add_definitions(-DOPEN_CV_INSTALLED)
set(COMMON_CXX_FLAGS "-std=c++11 -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_CXX_FLAGS} -fprofile-arcs -ftest-coverage -Wno-deprecated-declarations")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov")

include_directories(${PROJECT_SOURCE_DIR}/psdk/psdk_lib/include)
include_directories(${PROJECT_SOURCE_DIR}/psdk/sample_c++/module_sample)
include_directories(${PROJECT_SOURCE_DIR}/psdk/sample_c++/module_sample/liveview)
include_directories(${PROJECT_SOURCE_DIR}/psdk/sample_c++/module_sample/perception)
include_directories(${PROJECT_SOURCE_DIR}/psdk/sample_c++/platform/linux/common)
include_directories(${PROJECT_SOURCE_DIR}/psdk/sample_c++/platform/linux/manifold2/hal)
include_directories(${PROJECT_SOURCE_DIR}/psdk/sample_c++/platform/linux/manifold2/application)
include_directories(${PROJECT_SOURCE_DIR}/psdk/sample_c/platform/linux/common)
include_directories(${PROJECT_SOURCE_DIR}/psdk/sample_c/module_sample)

file(GLOB_RECURSE MODULE_SAMPLE_SRC
        ${PROJECT_SOURCE_DIR}/psdk/sample_c++/module_sample/liveview/*.c*
        ${PROJECT_SOURCE_DIR}/psdk/sample_c++/module_sample/perception/*.c*
        )
file(GLOB_RECURSE MODULE_HAL_SRC ${PROJECT_SOURCE_DIR}/psdk/sample_c++/platform/linux/manifold2/hal/*.c*)
file(GLOB_RECURSE MODULE_APP_SRC ${PROJECT_SOURCE_DIR}/psdk/sample_c++/platform/linux/manifold2/application/application.c*)

file(GLOB_RECURSE MODULE_COMMON_SRC_C ${PROJECT_SOURCE_DIR}/psdk/sample_c/platform/linux/common/*.c)
file(GLOB_RECURSE MODULE_SAMPLE_SRC_C ${PROJECT_SOURCE_DIR}/psdk/sample_c/module_sample/*.c)

execute_process(COMMAND uname -m
        OUTPUT_VARIABLE DEVICE_SYSTEM_ID)
if (DEVICE_SYSTEM_ID MATCHES x86_64)
    set(TOOLCHAIN_NAME x86_64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_x86_64=1)
elseif (DEVICE_SYSTEM_ID MATCHES aarch64)
    set(TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_aarch64=1)
else ()
    message(FATAL_ERROR "FATAL: Please confirm your platform.")
endif ()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/psdk/sample_c++/platform/linux/common/3rdparty)
find_package(FFMPEG REQUIRED)

if(FFMPEG_FOUND)
    message("Found FFMPEG FFMPEG_INCLUDE_DIR = ${FFMPEG_INCLUDE_DIR}")
    message("Found FFMPEG FFMPEG_LIBRARIES   = ${FFMPEG_LIBRARIES}")
else()
    message("Cannot Find FFMPEG")
endif(FFMPEG_FOUND)
include_directories(${FFMPEG_INCLUDE_DIR})

link_directories(${PROJECT_SOURCE_DIR}/psdk/psdk_lib/lib/${TOOLCHAIN_NAME})
link_libraries(${PROJECT_SOURCE_DIR}/psdk/psdk_lib/lib/${TOOLCHAIN_NAME}/libpayloadsdk.a -lstdc++)


#normal setting
include_directories(
    /usr/include/glib-2.0
    /usr/lib/aarch64-linux-gnu/glib-2.0/include
    /usr/local/include
    /usr/include/opencv4
    ./include/mqtt
)

link_directories(
    /usr/lib/aarch64-linux-gnu
    /usr/local/lib
    ./lib
)

find_package(Qt5 COMPONENTS Core Network Widgets Sql REQUIRED)

add_executable(${PROJECT_NAME}
    main.cpp
    vehiclecontrol.h
    vehiclecontrol.cpp
    global.h
    global.cpp
    LoopQueue.h
    analysis.h
    analysis.cpp
    applive.h
    applive.cpp
    tools.h
    flightControl.h
    flightControl.cpp
    mqttclient.h
    mqttclient.cpp
    httpclient.h
    httpclient.cpp
    ${MODULE_APP_SRC}
    ${MODULE_SAMPLE_SRC}
    ${MODULE_HAL_SRC}
    ${MODULE_SAMPLE_SRC_C}
    ${MODULE_COMMON_SRC_C}
)

target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Network Qt5::Widgets Qt5::Sql)
target_link_libraries(${PROJECT_NAME} glib-2.0)
target_link_libraries(${PROJECT_NAME} opencv_core opencv_imgproc opencv_highgui opencv_videoio opencv_imgcodecs opencv_dnn opencv_objdetect)
target_link_libraries(${PROJECT_NAME} sioclient curl paho-mqtt3a paho-mqtt3as)
target_link_libraries(${PROJECT_NAME} uuid)

#psdk
target_link_libraries(${PROJECT_NAME} m usb-1.0 ${FFMPEG_LIBRARIES})

find_package(OPUS REQUIRED)
if (OPUS_FOUND)
    message(STATUS "Found OPUS installed in the system")
    message(STATUS " - Includes: ${OPUS_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${OPUS_LIBRARY}")

    add_definitions(-DOPUS_INSTALLED)
    target_link_libraries(${PROJECT_NAME} ${OPUS_LIBRARY})
else ()
    message(STATUS "Cannot Find OPUS")
endif (OPUS_FOUND)

find_package(LIBUSB REQUIRED)
if (LIBUSB_FOUND)
    message(STATUS "Found LIBUSB installed in the system")
    message(STATUS " - Includes: ${LIBUSB_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${LIBUSB_LIBRARY}")

    add_definitions(-DLIBUSB_INSTALLED)
    target_link_libraries(${PROJECT_NAME} usb-1.0)
else ()
    message(STATUS "Cannot Find LIBUSB")
endif (LIBUSB_FOUND)
