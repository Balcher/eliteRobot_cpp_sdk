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

add_executable(mytest
    ${PROJECT_SOURCE_DIR}/test.cpp
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(
        mytest
        elite_robot_state
        pthread
    )
else()
    target_link_libraries(
        mytest
        elite_robot_state
    )
endif()


# socket通信测试
add_executable(socketConnect
    ${PROJECT_SOURCE_DIR}/socketConnect.cpp
    ${PROJECT_SOURCE_DIR}/Robot40011Port.cpp
    ${PROJECT_SOURCE_DIR}/Robot40011Port.hpp
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(
        socketConnect
        elite_robot_state
        pthread
    )
else()
    target_link_libraries(
        socketConnect
        elite_robot_state
    )
endif()

