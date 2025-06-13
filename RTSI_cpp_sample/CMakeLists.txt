cmake_minimum_required(VERSION 3.22.1)


project(eli_rtsi VERSION 0.0)

set(CMAKE_CXX_STANDARD 14)

include_directories(
    /usr/include/eigen3
)

add_library(
    ${PROJECT_NAME}
    STATIC
    Rtsi.cpp
    DataRecipe.cpp
    Logger/Logger.cpp
    rtsiReceive.cpp
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC ./
    PUBLIC ./Logger/
)

add_executable(
    rtsi_sample
    RtsiMain.cpp
)

add_executable(
    RtsiReceiveDemo
    RtsiReceiveDemo.cpp
    rtsiReceive.cpp
    rtsiReceive.h
)

add_compile_definitions(
    _WIN32_WINNT=0x0601
)

# 判断是否是 ROS 的 colcon 构建环境
if(DEFINED ENV{COLCON_PREFIX_PATH})
    # ROS colcon 构建环境
    message(STATUS "Building in ROS colcon environment, installing to ROS install directory.")

    # 安装库文件到 ROS 的 install/lib 目录
    install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib
    )

    # 安装头文件到 ROS 的 install/include 目录
    install(
        DIRECTORY ${PROJECT_SOURCE_DIR}/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.txt"
    )
else()
    # 非 ROS 环境
    message(STATUS "Building in non-ROS environment, using default CMake install paths.")

    # 安装库文件到默认的 CMake 安装路径
    install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    # 安装头文件到默认的 CMake 安装路径
    install(
        DIRECTORY ${PROJECT_SOURCE_DIR}/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.txt"
    )
endif(WIN32)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(
        rtsi_sample
        ${PROJECT_NAME}
        pthread
    )
    target_link_libraries(
        RtsiReceiveDemo
        ${PROJECT_NAME}
        pthread
    )
else()
    target_link_libraries(
        rtsi_sample
        ${PROJECT_NAME}
    )
endif()
