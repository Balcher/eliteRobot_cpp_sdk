# eliteRobot_cpp_sdk
## 艾利特CS系列机械臂cpp版本的SDK

### 简介

解析 Elite CS 系列机器人 30001 端口的 RobotState 报文。

### 编译标准与依赖

+ C++14标准
+ boost1.82 中的 asio

### 如何编译

1. 安装cmake，且cmake版本不低于为3.22.1

3. mkdir build

4. cmake ..

5. cmake --build .

### 代码说明

详见doc目录下



## 手眼标定

2025.1.24 ：添加手眼标定相关的简单实现，包括眼在手上，眼在手外
