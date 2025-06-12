cmake_minimum_required(VERSION 3.22.1)


project(eli_rtsi VERSION 0.0)

set(CMAKE_CXX_STANDARD 14)


add_library(
    ${PROJECT_NAME}
    STATIC
    Rtsi.cpp
    DataRecipe.cpp
    Logger/Logger.cpp
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
