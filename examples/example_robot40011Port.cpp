#include "Robot40011Port.hpp"
#include <iostream>
#include <vector>
#include <memory>

int main()
{
    std::string ip = "192.168.205.133";
    auto robot = std::make_unique<Robot40011Port>();

    if(!robot->connect(ip, 40011)) {
        std::cerr << "Connect failed" << std::endl;
        return -1;
    }

    // 示例位姿
    std::vector<double> targetPose = {0.3, 0.2, 0.5, 3.14, 0.0, 0.0};
    std::vector<double> jointDegrees(6);

    // 获取逆解大概需要15ms左右
    if(robot->getIKDegrees(targetPose, jointDegrees)) {
        std::cout << "IK Degrees: ";
        for(auto d : jointDegrees) std::cout << d << "\t";
        std::cout << std::endl;

        // 使用 IK 得到的关节角度查询 FK
        std::vector<double> fkPose(6);
        if(robot->getFKPose(jointDegrees, fkPose)) {
            std::cout << "FK Pose: ";
            for(auto p : fkPose) std::cout << p << "\t";
            std::cout << std::endl;
        }
    } else {
        std::cout << "No IK solution found" << std::endl;
    }

    robot->disconnect();
    return 0;
}


