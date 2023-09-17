cmake_minimum_required(VERSION 2.8)

project(liveTool)

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -pthread -g -O0")


find_package(Qt5 COMPONENTS Core Network Widgets REQUIRED)


add_executable(${PROJECT_NAME}
    main.cpp
    app.h
    app.cpp
    monitor.h
    monitor.cpp
)

target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Network Qt5::Widgets)
