cmake_minimum_required(VERSION 3.22)
project(UserUseDemo)

if(UNIX)
    set(CMAKE_PREFIX_PATH "/usr/local")    # 这里改成自己的安装路径，按照默认cmake的话应该是在/usr/local
    find_package(elite_robot_SDK REQUIRED)
    add_executable(main main.cpp)
    target_link_libraries(main elite_robot_SDK)
    target_compile_definitions(main PRIVATE CONFIG_PATH=\"${CMAKE_PREFIX_PATH}/share/RobotState/config/\")    # 这里是加载机械臂要用的那个txt
endif()

