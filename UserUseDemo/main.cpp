#include "RobotInterface.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <cstdio>

int main(int argc, char **argv)
{
    std::unique_ptr<RobotInterface> robot = std::make_unique<RobotInterface>(); // 创建机器人接口实例

    // IP 地址可以通过命令行指定，默认值
    std::string ipAdd = (argc >= 2) ? argv[1] : "192.168.205.133";

    // 构造配置文件路径
    std::string cfg_file = std::string(CONFIG_PATH) + "CS_UserManual_Robot_State_Message.txt";
    std::cout << "Loading config file: " << cfg_file << std::endl;

    // 加载配置文件
    if (!robot->loadConfigure(cfg_file))
    {
        std::cerr << "Load Configure file failed. Check file path." << std::endl;
        return -1;
    }

    // 设置异常回调
    robot->setExceptionCallback([](const RobotException &exception) {
        std::cout << "time stamp: " << exception.timestamp << " ";
        if (exception.exception_type == RobotException::ExceptionType::RUN_TIME_EXCEPTION)
            std::cout << "Runtime exception" << std::endl;
        else if (exception.exception_type == RobotException::ExceptionType::ERROR_CODE)
            std::cout << "Robot error exception" << std::endl;
    });

    // 尝试连接机器人
    if (!robot->connect(ipAdd, 30001))
    {
        std::cerr << "Connect failed. Check network or IP." << std::endl;
        return -1;
    }
    std::cout << "Connected to robot at " << ipAdd << std::endl;

    // 发送故意引发运行时异常的脚本测试异常处理
    robot->sendScript("def func():\n\tabcd(123)\nend\n");

    // 循环获取机器人状态
    while (robot->isConnect())
    {
        std::vector<double> joint = robot->getJointActualPos();    // 关节位置
        std::vector<float> jointTorque = robot->getJointTorques(); // 力矩
        std::vector<double> tcpOffset = robot->getTcpOffset();     // TCP 偏移量

        // 打印关节位置
        for (auto item : joint)
            printf("%.3lf\t", item);
        printf("\n");

        // 可选：打印力矩和 TCP 偏移量
        // for (auto item : jointTorque) printf("%.3lf\t", item);
        // for (auto item : tcpOffset) printf("%.3lf\t", item);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Robot disconnected. Exiting." << std::endl;
    return 0;
}
