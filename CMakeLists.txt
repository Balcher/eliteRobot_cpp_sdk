cmake_minimum_required(VERSION 3.22.1)
project(RobotState)

set(CMAKE_CXX_STANDARD 17)
include(GNUInstallDirs)

if(WIN32)
    include_directories("D:\\Program Files (x86)\\boost_1_82_0")

    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    add_compile_definitions(_WIN32_WINNT=0x0601)
    endif()
elseif(UNIX)

endif()


if(WIN32)
    # Windows 手动指定 Eigen 路径
    include_directories("D:/Libraries/eigen-3.4.0")
elseif(UNIX)
    # Linux 用 find_package
    find_package(Eigen3 3.4 REQUIRED NO_MODULE)
    if(Eigen3_FOUND)
        message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIRS}")
    endif()
    include_directories(${EIGEN3_INCLUDE_DIRS})
endif()


# -------------------------------
# 源文件
# -------------------------------
set(
    SOURCE_FILE
    src/RobotStateLoadFile.cpp
    src/RobotState.cpp
    src/RobotException.cpp
    src/RobotInterface.cpp
    src/Robot40011Port.cpp
    src/Rtsi.cpp
    src/DataRecipe.cpp
    src/rtsiReceive.cpp
    src/Logger.cpp
)

add_library(
    elite_robot_SDK
    STATIC
    "${SOURCE_FILE}"
)

# -------------------------------
# 头文件包含路径
# -------------------------------
target_include_directories(
    elite_robot_SDK
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# -------------------------------
# 链接依赖
# -------------------------------
if(WIN32)
    target_link_libraries(elite_robot_SDK PRIVATE ws2_32)
else(UNIX)
    target_link_libraries(elite_robot_SDK PRIVATE pthread)
endif()

# -------------------------------
# 安装配置
# -------------------------------
# 库文件安装
install(TARGETS elite_robot_SDK
    EXPORT elite_robot_SDKTargets                 # 导出target
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}     # 静态库
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}     # 动态库
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}     # 可执行文件 (Windows .dll/.exe)
)

# 头文件安装
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# 配置文件单独安装到 share/${PROJECT_NAME}/config
install(DIRECTORY ${PROJECT_SOURCE_DIR}/config/
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.txt"
)

# 编译选项
option(BUILD_EXAMPLES "Build examples" ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

install(EXPORT elite_robot_SDKTargets
    FILE elite_robot_SDKConfig.cmake       # 安装时生成 find_package 使用的 cmake 文件
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/elite_robot_SDK
)

