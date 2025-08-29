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

add_library(
    elite_robot_state
    STATIC
    "${SOURCE_FILE}"
)

target_include_directories(
    elite_robot_state
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# add winsock
if(WIN32)
    target_link_libraries(elite_robot_state PRIVATE ws2_32)
endif()

set(INSTALL_LIB_DIR "lib" CACHE PATH "Directory for installing libraries")
set(INSTALL_INCLUDE_DIR "include" CACHE PATH "Directory for installing header files")

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
        DIRECTORY ${PROJECT_SOURCE_DIR}/include
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.txt"
    )
else()
    # 非 ROS 环境
    message(STATUS "Building in non-ROS environment, using default CMake install paths.")

    # 安装库文件到默认的 CMake 安装路径
    install(
        TARGETS elite_robot_state
        # ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        # LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        # RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
    )

    # 安装头文件到默认的 CMake 安装路径
    install(
        DIRECTORY ${PROJECT_SOURCE_DIR}/include
        # DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        DESTINATION ${INSTALL_INCLUDE_DIR}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )

    install(FILES
        ${PROJECT_SOURCE_DIR}/RobotState.hpp
        ${PROJECT_SOURCE_DIR}/RobotException.hpp
        ${PROJECT_SOURCE_DIR}/RobotInterface.hpp
        ${PROJECT_SOURCE_DIR}/Robot40011Port.hpp
        ${PROJECT_SOURCE_DIR}/CS_UserManual_Robot_State_Message.txt
        DESTINATION ${INSTALL_INCLUDE_DIR}
    )
endif()

option(BUILD_DEMOS "Build demo/test programs" OFF)

if(BUILD_DEMOS)
    add_subdirectory(${PROJECT_SOURCE_DIR}/CppSample)
    add_subdirectory(${PROJECT_SOURCE_DIR}/HandEyeCalib)
    add_subdirectory(${PROJECT_SOURCE_DIR}/RTSI_cpp_sample)
endif()
