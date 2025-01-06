#include "RobotInterface.hpp"

#include <stdio.h>
#include <iostream>
#include <memory>

int main(int argc, char** argv) {
    std::unique_ptr<RobotInterface> robot = std::make_unique<RobotInterface>();    // ���������˽ӿ�ʵ��
    //if (argc < 2) {
    //    std::cout << "Need provide Elite Robot IP address" << std::endl;           // ���û���ṩ������IP��ַ���������ʾ��Ϣ�����ش���
    //    return -1;
    //}

    std::string ipAdd = "192.168.205.132";                                         // ip��ַ
    if (!robot->loadConfigure("CS_UserManual_Robot_State_Message.txt")) {          // ���������ļ�
        std::cout << "Load Configure file fail. Check file path" << std::endl;
        return -1;
    }
    robot->setExceptionCallback([](const RobotException& exception) {               // �쳣����ص�
        // do somethings when an exception occurs.
        std::cout << "time stamp: " << exception.timestamp << " ";
        if (exception.exception_type == RobotException::ExceptionType::RUN_TIME_EXCEPTION) {
            std::cout << "Runtime exception" << std::endl;
        }
        else if (exception.exception_type == RobotException::ExceptionType::ERROR_CODE) {
            std::cout << "Robot error exception" << std::endl;
        }
        });
    if (!robot->connect(ipAdd, 30001)) {                                          // ���ӵ�������
        std::cout << "Connect fail check network or IP" << std::endl;
        return -1;
    }
    // Make a robot runtime exception
    //robot->sendScript("def func():\n\tabcd(123)\nend\n");                         // ����һ��������������ʱ�쳣�Ľű�
    std::vector<double> joint;

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    std::vector<double> angles = { -3.14,-1.57,-1.57,-1.57,1.57,0 };
    double acceleration = 1.4;
    double velocity = 0.5;
    double time = 0;
    double r = 0;

    robot->robotMove(angles, acceleration, velocity, time, r);


    while (true) {

        // �ж��Ƿ�����
        if (!robot->isConnect()) {
            return 0;
        }
        joint = robot->getJointActualPos();                                       // ��ȡ�����˵�λ��

        // ��ȡ��ӡ�����˹ؽ�λ��
        for (auto item : joint) {
            printf("%.3lf\t", item);
        }
        printf("\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}