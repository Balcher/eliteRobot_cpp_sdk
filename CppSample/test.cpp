#include "RobotInterface.hpp"

#include <stdio.h>
#include <iostream>
#include <memory>

int main(int argc, char** argv) {
    std::unique_ptr<RobotInterface> robot = std::make_unique<RobotInterface>();    // 创建机器人接口实例
    //if (argc < 2) {
    //    std::cout << "Need provide Elite Robot IP address" << std::endl;           // 如果没有提供机器人IP地址，则输出提示信息并返回错误
    //    return -1;
    //}

    std::string ipAdd = "192.168.1.133";                                         // ip地址
    if (!robot->loadConfigure("CS_UserManual_Robot_State_Message.txt")) {          // 加载配置文件
        std::cout << "Load Configure file fail. Check file path" << std::endl;
        return -1;
    }
    robot->setExceptionCallback([](const RobotException& exception) {               // 异常处理回调
        // do somethings when an exception occurs.
        std::cout << "time stamp: " << exception.timestamp << " ";
        if (exception.exception_type == RobotException::ExceptionType::RUN_TIME_EXCEPTION) {
            std::cout << "Runtime exception" << std::endl;
        }
        else if (exception.exception_type == RobotException::ExceptionType::ERROR_CODE) {
            std::cout << "Robot error exception" << std::endl;
        }
        });
    if (!robot->connect(ipAdd, 30001)) {                                          // 连接到机器人
        std::cout << "Connect fail check network or IP" << std::endl;
        return -1;
    }
    // Make a robot runtime exception
    //robot->sendScript("def func():\n\tabcd(123)\nend\n");                         // 发送一个故意引发运行时异常的脚本
    std::vector<double> joint;

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    std::vector<double> angles = { -3.14,-1.57,-1.57,-1.57,1.57,0 };
    double acceleration = 1.4;
    double velocity = 0.5;
    double time = 0;
    double r = 0;

    robot->robotMove(angles, acceleration, velocity, time, r);

    /////////////////////////////////////////////////////// 
    // 输出DH参数表
    std::vector<double> a_i = robot->getDhAJoint();
    std::vector<double> d_i = robot->getDhDJoint();
    std::vector<double> alpha_i = robot->getDhAlphaJoint();
    std::cout << "----- 打印DH参数表 -----" << std::endl;
    std::cout << "alpha \t a \t d \t\n" << std::endl;
    for (int i = 0; i < a_i.size(); i++)
    {
        printf("%.3lf \t %.3lf \t %.3lf \t\n", alpha_i[i], a_i[i], d_i[i]);
    }

    // 获取姿态位置
    std::vector<double> TcpPositions = robot->getTcpPosition();
    std::cout << "x  y  z(m)  rx  ry  rz(rad)" << std::endl;
    for (auto TcpPosition : TcpPositions)
    {
        std::cout << TcpPosition << " ";
    }
    std::cout << std::endl;

    //////////////////////////////////////////////////////
    // 正解计算


    //////////////////////////////////////////////////////

    while (true) {

        // 判断是否连接
        if (!robot->isConnect()) {
            return 0;
        }
        joint = robot->getTcpPosition();                                       // 获取机器人的TCP姿态

        // 获取打印机器人关节位置
        for (auto item : joint) {
            printf("%.3lf\t", item);
        }
        printf("\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}