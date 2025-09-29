#include "RobotInterface.hpp"

#include <iostream>
#include <memory>
#include <vector>
#include <thread>
#include <chrono>
#include <cstdio>
#include <unistd.h>

int main(int argc, char** argv) {
    // 创建机器人接口实例
    std::unique_ptr<RobotInterface> robot = std::make_unique<RobotInterface>();

    // IP 地址可通过命令行参数指定，否则使用默认值
    std::string ipAdd = (argc >= 2) ? argv[1] : "192.168.205.133";

    // 加载配置文件
    std::string cfg_file = std::string(CONFIG_PATH) + "CS_UserManual_Robot_State_Message.txt";
    std::cout << "Loading config file: " << cfg_file << std::endl;

    if (!robot->loadConfigure(cfg_file)) {
        std::cerr << "Load Configure file failed. Check file path." << std::endl;
        return -1;
    }

    // 设置异常回调
    robot->setExceptionCallback([](const RobotException& exception) {
        std::cout << "time stamp: " << exception.timestamp << " ";
        if (exception.exception_type == RobotException::ExceptionType::RUN_TIME_EXCEPTION)
            std::cout << "Runtime exception" << std::endl;
        else if (exception.exception_type == RobotException::ExceptionType::ERROR_CODE)
            std::cout << "Robot error exception" << std::endl;
    });

    // 连接机器人
    if (!robot->connect(ipAdd, 30001)) {
        std::cerr << "Connect failed. Check network or IP." << std::endl;
        return -1;
    }
    std::cout << "Connected to robot at " << ipAdd << std::endl;

    // 等待机器人就绪
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // 设置关节目标位置并运动
    std::vector<double> angles = { 0, -1.81, -0.9, -1.57, 1.57, 0 };    // 关节角度值 rad
    double acceleration = 1.4;
    double velocity = 0.5;
    double time = 0;
    double r = 0;
    robot->robotJointMove(angles, acceleration, velocity, time, r);

    sleep(5);

    // 直线运动形式
    std::vector<double> targetPose = {0.73998, -0.17250, 1.75390, -2.246, 0.0, -1.571};    // 位姿形式 x y z(m) rx ry rz(rad)
    robot->robotLinearMove(targetPose, acceleration, velocity, time, r);

    // 输出DH参数表
    std::vector<double> a_i = robot->getDhAJoint();
    std::vector<double> d_i = robot->getDhDJoint();
    std::vector<double> alpha_i = robot->getDhAlphaJoint();
    std::cout << "----- DH Parameters -----" << std::endl;
    std::cout << "alpha \t a \t d" << std::endl;
    for (size_t i = 0; i < a_i.size(); ++i) {
        printf("%.3lf \t %.3lf \t %.3lf\n", alpha_i[i], a_i[i], d_i[i]);
    }

    // 获取并打印 TCP 姿态
    std::vector<double> TcpPositions = robot->getTcpPosition();
    std::cout << "TCP Position: x y z(m) rx ry rz(rad)" << std::endl;
    for (auto val : TcpPositions) {
        std::cout << val << " ";
    }
    std::cout << std::endl;

    // 循环实时获取 TCP 姿态
    while (robot->isConnect()) {
        std::vector<double> tcp = robot->getTcpPosition();
        for (auto val : tcp) {
            printf("%.3lf\t", val);
        }
        printf("\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cout << "Robot disconnected. Exiting." << std::endl;
    return 0;
}
