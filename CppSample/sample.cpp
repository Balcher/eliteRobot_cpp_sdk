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

    std::string ipAdd = "192.168.205.133";                                         // ip地址
    if (!robot->loadConfigure("CS_UserManual_Robot_State_Message.txt")) {          // 加载配置文件
        std::cout << "Load Configure file fail. Check file path" << std::endl;
        return -1;
    }
    robot->setExceptionCallback([](const RobotException& exception){               // 异常处理回调
        // do somethings when an exception occurs.
        std::cout << "time stamp: " << exception.timestamp << " ";
        if (exception.exception_type == RobotException::ExceptionType::RUN_TIME_EXCEPTION) {
            std::cout << "Runtime exception" << std::endl;
        } else if (exception.exception_type == RobotException::ExceptionType::ERROR_CODE) {
            std::cout << "Robot error exception" << std::endl;
        }
    });
    if (!robot->connect(ipAdd, 30001)) {                                          // 连接到机器人
        std::cout << "Connect fail check network or IP" << std::endl;
        return -1;
    }
    // Make a robot runtime exception
    robot->sendScript("def func():\n\tabcd(123)\nend\n");                         // 发送一个故意引发运行时异常的脚本
    std::vector<double> joint;

    while (true) {

        // 判断是否连接
        if (!robot->isConnect()) {
            return 0;
        }
        joint = robot->getJointActualPos();                                       // 获取机器人的位姿
        
        // 获取打印机器人关节位置
        for (auto item : joint) {
            printf("%.3lf\t", item);
        }
        printf("\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}