#include "RobotInterface.hpp"

#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include "Robot40011Port.hpp"

int main()
{

    // 40011端口
    std::unique_ptr<Robot40011Port> robot = std::make_unique<Robot40011Port>(); // 创建机器人接口实例
    std::string ipAdd = "192.168.205.133";                                      // ip地址

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

    // // 逆解运算
    // std::vector<double> poses = {-0.39527, 0.20718, 0.23822, 3.137, -0.018, 1.134}; // 复位摆放位置
    // // std::vector<double> poses = { 1.20282, -0.12892, 1.07623, -3.141, -0.001, -2.042 }; // 准备抓取位置

    // std::vector<double> degrees;
    // bool flag = robot->getIKDegrees(poses, degrees);
    // std::cout << "flag = " << flag << std::endl;

    // // 正解运算
    // std::vector<double> poses2;
    // bool flag1 = robot->getFKPose(degrees, poses2);
    // if (flag1 == true)
    // {
    //     for (auto poses : poses2)
    //     {
    //         std::cout << "poses = " << poses << std::endl;
    //     }
    // }
    // else
    // {
    //     std::cout << "flag1 = " << flag1 << std::endl;
    // }

    // // for (auto degree : degrees)
    // // {
    // //     std::cout << "degree = " << degree << std::endl;
    // // }

    // // // 执行运动
    // // if (flag == true)
    // // {
    // //     double acceleration = 1.4;
    // //     double velocity = 0.5;
    // //     double time = 0;
    // //     double r = 0;

    // //     robot2->robotJointMove(degrees, acceleration, velocity, time, r);
    // // }

    ////////////////////////////////给位姿控制///////////////////////////////////////////
    // std::vector<double> posesReset = {-0.48418, 0.14341, 0.21646, -3.130, 0.001, 1.629};  // 复位摆放位置
    // std::vector<double> moveToGrasp = {0.16062, 0.47875, 0.21646, -3.130, 0.001, 0.023};  // 从复位点移动出来
    // std::vector<double> readyToGrasp = {1.25961, -0.17224, 0.87499, -3.141, 0.0, -1.571}; // 准备抓取位置
    // double acceleration = 1.4;
    // double velocity = 0.5;
    // double time = 0;
    // double r = 0;
    // robot2->robotLinearMove(posesReset, acceleration, velocity, time, r);    // 复位摆放位置
    // sleep(10);

    // robot2->robotLinearMove(moveToGrasp, acceleration, velocity, time, r);    // 从复位点移动出来
    // sleep(3);
    // robot2->robotLinearMove(readyToGrasp, acceleration, velocity, time, r);    // 准备抓取位置

    ////////////////////////////流程演示//////////////////////////////////////////
    std::vector<double> posesReset = {-0.48418, 0.14341, 0.21646, -3.130, 0.001, 1.629};  // 复位摆放位置
    std::vector<double> moveToGrasp = {0.16062, 0.47875, 0.21646, -3.130, 0.001, 0.023};  // 从复位点移动出来
    std::vector<double> readyToGrasp = {1.25961, -0.17224, 0.87499, -3.141, 0.0, -1.571}; // 准备抓取位置
    double acceleration = 1.4;
    double velocity = 0.5;
    double time = 0;
    double r = 0;

    // 1. 复位摆放位置
    // 计算逆解
    std::vector<double> degrees;
    bool flag = robot->getIKDegrees(posesReset, degrees);
    if (flag != true)
    {
        std::cout << "到不了" << flag << std::endl;
        return -1;
    }
    // 移动
    robot2->robotJointMove(degrees, acceleration, velocity, time, r); // 复位摆放位置
    sleep(5);

    // 2. 从复位点移动出来
    // 计算逆解
    degrees.clear();
    flag = robot->getIKDegrees(moveToGrasp, degrees);
    if (flag != true)
    {
        std::cout << "到不了" << flag << std::endl;
        return -1;
    }
    // 移动
    robot2->robotJointMove(degrees, acceleration, velocity, time, r); // 从复位点移动出来
    sleep(5);

    // 3. 准备抓取位置
    // 计算逆解
    degrees.clear();
    flag = robot->getIKDegrees(readyToGrasp, degrees);
    if (flag != true)
    {
        std::cout << "到不了" << flag << std::endl;
        return -1;
    }
    // 移动
    robot2->robotJointMove(degrees, acceleration, velocity, time, r); // 准备抓取位置
    sleep(5);

    // 4. 自定义一个货物的位置
    // 定义位置
    std::vector<double> cartonPosition = {1.05441, -0.15250, -0.74005, -3.138, -0.002, -1.065}; // 货物地方

    // 5. 移动到货物上方1m处
    std::vector<double> cartonPosition_05m = cartonPosition;
    cartonPosition_05m[2] += 0.5; // 上方0.5m处
    // 计算逆解
    degrees.clear();
    flag = robot->getIKDegrees(cartonPosition_05m, degrees);
    if (flag != true)
    {
        std::cout << "到不了" << flag << std::endl;
        return -1;
    }
    // 移动
    robot2->robotJointMove(degrees, acceleration, velocity, time, r); // 货物上方一米
    sleep(5);

    // 6. 到达抓取货物的地方
    robot2->robotLinearMove(cartonPosition, acceleration, velocity, time, r); // 直线运动到货物地方
    sleep(5);

    // 7. 返回到货物上方1m处
    robot2->robotLinearMove(cartonPosition_05m, acceleration, velocity, time, r); // 货物上方一米
    sleep(5);

    // 9. 移动到准备抓取位置
    degrees.clear();
    flag = robot->getIKDegrees(readyToGrasp, degrees);
    if (flag != true)
    {
        std::cout << "到不了" << flag << std::endl;
        return -1;
    }
    // 移动
    robot2->robotJointMove(degrees, acceleration, velocity, time, r); // 准备抓取位置
    sleep(5);

    // 10. 复位
    // 计算逆解
    degrees.clear();
    flag = robot->getIKDegrees(posesReset, degrees);
    if (flag != true)
    {
        std::cout << "到不了" << flag << std::endl;
        return -1;
    }
    // 移动
    robot2->robotJointMove(degrees, acceleration, velocity, time, r); // 复位摆放位置
}
