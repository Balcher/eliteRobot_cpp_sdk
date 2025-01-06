cmake_minimum_required(VERSION 3.22.1)

project(RobotState)

set(CMAKE_CXX_STANDARD 17)

include_directories("D:\\Program Files (x86)\\boost_1_82_0")

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    add_compile_definitions(_WIN32_WINNT=0x0601)
endif()

set(
    SOURCE_FILE
    RobotStateLoadFile.cpp
    RobotState.cpp
    RobotException.cpp
    RobotInterface.cpp
)

include_directories(
    ${PROJECT_SOURCE_DIR}
)

add_library(
    elite_robot_state
    STATIC
    "${SOURCE_FILE}"
)



add_subdirectory(${PROJECT_SOURCE_DIR}/CppSample/)