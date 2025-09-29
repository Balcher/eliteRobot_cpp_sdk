set(CONFIG_PATH_STR "${CMAKE_INSTALL_PREFIX}/share/RobotState/config/")
string(CONFIGURE "\"${CONFIG_PATH_STR}\"" CONFIG_PATH_CPP)

# sample_robot_state
add_executable(sample_robot_state sample_robot_state.cpp)
target_include_directories(sample_robot_state PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(sample_robot_state PRIVATE elite_robot_SDK)
target_compile_definitions(sample_robot_state PRIVATE CONFIG_PATH=${CONFIG_PATH_CPP})

# example_robot_control
add_executable(example_robot_control example_robot_control.cpp)
target_include_directories(example_robot_control PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(example_robot_control PRIVATE elite_robot_SDK)
target_compile_definitions(example_robot_control PRIVATE CONFIG_PATH=${CONFIG_PATH_CPP})

# example_robot40011Port
add_executable(example_robot40011Port example_robot40011Port.cpp)
target_include_directories(example_robot40011Port PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(example_robot40011Port PRIVATE elite_robot_SDK)
target_compile_definitions(example_robot40011Port PRIVATE CONFIG_PATH=${CONFIG_PATH_CPP})

# example_realTime_fkik
add_executable(example_realTime_fkik example_realTime_fkik.cpp)
target_include_directories(example_realTime_fkik PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(example_realTime_fkik PRIVATE elite_robot_SDK)
target_compile_definitions(example_realTime_fkik PRIVATE CONFIG_PATH=${CONFIG_PATH_CPP})

# example_RtsiReceiveDemo
add_executable(example_RtsiReceiveDemo example_RtsiReceiveDemo.cpp)
target_include_directories(example_RtsiReceiveDemo PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(example_RtsiReceiveDemo PRIVATE elite_robot_SDK)