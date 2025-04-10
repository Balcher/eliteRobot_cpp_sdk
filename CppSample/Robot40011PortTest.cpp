#include "RobotInterface.hpp"

#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include "Robot40011Port.hpp"

int main()
{

    // 40011端口
    std::unique_ptr<Robot40011Port> robot = std::make_unique<Robot40011Port>(); // 创建机器人接口实例
    std::string ipAdd = "192.168.1.133";                                        // ip地址

    if (!robot->connect(ipAdd, 40011))
    {
        std::cout << "Connect fail check network or IP" << std::endl;
        return -1;
    }

    // 30001端口
    std::unique_ptr<RobotInterface> robot2 = std::make_unique<RobotInterface>(); // 创建机器人接口实例

    if (!robot2->loadConfigure("CS_UserManual_Robot_State_Message.txt"))
    { // 加载配置文件
        std::cout << "Load Configure file fail. Check file path" << std::endl;
        return -1;
    }
    robot2->setExceptionCallback([](const RobotException &exception) { // 异常处理回调
        // do somethings when an exception occurs.
        std::cout << "time stamp: " << exception.timestamp << " ";
        if (exception.exception_type == RobotException::ExceptionType::RUN_TIME_EXCEPTION)
        {
            std::cout << "Runtime exception" << std::endl;
        }
        else if (exception.exception_type == RobotException::ExceptionType::ERROR_CODE)
        {
            std::cout << "Robot error exception" << std::endl;
        }
    });
    if (!robot2->connect(ipAdd, 30001))
    { // 连接到机器人
        std::cout << "Connect fail check network or IP" << std::endl;
        return -1;
    }

    // 逆解运算
    std::vector<double> poses = {-0.39527, 0.20718, 0.23822, 3.137, -0.018, 1.134}; // 复位摆放位置
    // std::vector<double> poses = { 1.20282, -0.12892, 1.07623, -3.141, -0.001, -2.042 }; // 准备抓取位置

    // std::vector<double> degrees;
    // bool flag = robot->getIKDegrees(poses, degrees);
    // std::cout << "flag = " << flag << std::endl;
    // for (auto degree : degrees)
    // {
    //     std::cout << "degree = " << degree << std::endl;
    // }

    // // 执行运动
    // if (flag == true)
    // {
    //     double acceleration = 1.4;
    //     double velocity = 0.5;
    //     double time = 0;
    //     double r = 0;

    //     robot2->robotJointMove(degrees, acceleration, velocity, time, r);
    // }

    ////////////////////////////////直接给角度控制///////////////////////////////////////////
    // 准备抓取的位置（弧度）
    // degree = 0.036648
    // degree = -1.91636
    // degree = -1.05379
    // degree = -1.74221
    // degree = 1.56963
    // degree = 0.507851

    // 为避免膨胀，机械臂大轴先旋转出来
    // degree = 1.72
    // degree = -1.91636
    // degree = -1.05379
    // degree = -1.74221
    // degree = 1.56963
    // degree = 0.507851

    // 复位位置（弧度）
    // degree = 3.06107
    // degree = -1.67699
    // degree = -2.79035
    // degree = -0.25564
    // degree = 1.55553
    // degree = 0.356391

    std::vector<double> degreeReset = { 3.06107, -1.67699, -2.79035, -0.25564, 1.55553,  0.356391}; // 复位
    std::vector<double> degreeGrasp1 = { 1.72, -1.67699, -2.79035, -0.25564, 1.55553,  0.356391}; // 准备抓取，J1旋转出来
    std::vector<double> degreeGrasp2 = { 0.036648, -1.91636, -1.05379, -1.74221,  1.56963,  0.507851}; // 准备抓取位置

    double acceleration = 1.4;
    double velocity = 0.5;
    double time = 0;
    double r = 0;
    robot2->robotJointMove(degreeReset, acceleration, velocity, time, r);    // 复位
    sleep(10);

    robot2->robotJointMove(degreeGrasp1, acceleration, velocity, time, r);    // 复位
    sleep(3);
    robot2->robotJointMove(degreeGrasp2, acceleration, velocity, time, r);    // 复位

    ///////////////////////////////////////////////////////////////////////
}
