cmake_minimum_required(VERSION 3.22.1)

project(HandEyeCalib)

set(CMAKE_CXX_STANDARD 17)
if(WIN32)
    set(OpenCV_DIR "D:/opencv4.8/opencv/build/x64/vc16/lib")
    find_package(OpenCV 4.8 REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})

    include_directories("D:\\Program Files (x86)\\eigen-3.4.0")
    add_executable(calibExample calibExample.cpp "handEyeCalib.cpp" "handEyeCalib.hpp")
    target_link_libraries(calibExample ${OpenCV_LIBS})
elseif(UNIX)
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})

    include_directories("/usr/include/eigen3")
    add_executable(calibExample calibExample.cpp "handEyeCalib.cpp" "handEyeCalib.hpp")
    target_link_libraries(calibExample ${OpenCV_LIBS})
endif()