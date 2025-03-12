cmake_minimum_required(VERSION 3.22.1)

project(RobotState)

set(CMAKE_CXX_STANDARD 17)
if(WIN32)
    include_directories("D:\\Program Files (x86)\\boost_1_82_0")

    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    add_compile_definitions(_WIN32_WINNT=0x0601)
    endif()
elseif(UNIX)

endif()


set(
    SOURCE_FILE
    RobotStateLoadFile.cpp
    RobotState.cpp
    RobotException.cpp
    RobotInterface.cpp
    Robot40011Port.cpp
)

include_directories(
    ${PROJECT_SOURCE_DIR}
)

add_library(
    elite_robot_state
    STATIC
    "${SOURCE_FILE}"
)

# 判断是否是 ROS 的 colcon 构建环境
if(DEFINED ENV{COLCON_PREFIX_PATH})
    # ROS colcon 构建环境
    message(STATUS "Building in ROS colcon environment, installing to ROS install directory.")

    # 安装库文件到 ROS 的 install/lib 目录
    install(
        TARGETS elite_robot_state
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib
    )

    # 安装头文件到 ROS 的 install/include 目录
    install(
        DIRECTORY ${PROJECT_SOURCE_DIR}/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )
else()
    # 非 ROS 环境
    message(STATUS "Building in non-ROS environment, using default CMake install paths.")

    # 安装库文件到默认的 CMake 安装路径
    install(
        TARGETS elite_robot_state
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

    # 安装头文件到默认的 CMake 安装路径
    install(
        DIRECTORY ${PROJECT_SOURCE_DIR}/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )
endif()

add_subdirectory(${PROJECT_SOURCE_DIR}/CppSample/)
add_subdirectory(${PROJECT_SOURCE_DIR}/HandEyeCalib/)