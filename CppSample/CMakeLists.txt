cmake_minimum_required(VERSION 3.22.1)

project(CppSample)

set(CMAKE_CXX_STANDARD 17)

include_directories("D:\\Program Files (x86)\\boost_1_82_0")

configure_file(
    ${PROJECT_SOURCE_DIR}/../CS_UserManual_Robot_State_Message.txt
    ${PROJECT_BINARY_DIR}/CS_UserManual_Robot_State_Message.txt
    COPYONLY
)
add_executable(
    ${PROJECT_NAME}
    ${PROJECT_SOURCE_DIR}/sample.cpp
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(
        ${PROJECT_NAME}
        elite_robot_state
        pthread
    )
else()
    target_link_libraries(
        ${PROJECT_NAME}
        elite_robot_state
    )
endif()

# socket通信测试
add_executable(Robot40011PortTest
    ${PROJECT_SOURCE_DIR}/Robot40011PortTest.cpp
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(
        Robot40011PortTest
        elite_robot_state
        pthread
    )
else()
    target_link_libraries(
        Robot40011PortTest
        elite_robot_state
    )
endif()

# 打印机器人关节姿态
add_executable(test
    ${PROJECT_SOURCE_DIR}/test.cpp
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(
        test
        elite_robot_state
        pthread
    )
else()
    target_link_libraries(
        test
        elite_robot_state
    )
endif()

# movel控制方式测试
add_executable(movel_test
    ${PROJECT_SOURCE_DIR}/movel_test.cpp
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(
        movel_test
        elite_robot_state
        pthread
    )
else()
    target_link_libraries(
        movel_test
        elite_robot_state
    )
endif()