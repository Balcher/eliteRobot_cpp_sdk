#include "RobotInterface.hpp"
#include "Robot40011Port.hpp"

#include <iostream>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>

int main()
{
    std::string ip = "192.168.205.133";

    // ------------------ 40011端口 ------------------
    auto robot40011 = std::make_unique<Robot40011Port>();
    if(!robot40011->connect(ip, 40011))
    {
        std::cerr << "Failed to connect 40011 port" << std::endl;
        return -1;
    }

    // ------------------ 30001端口 ------------------
    auto robotInterface = std::make_unique<RobotInterface>();
    std::string cfg_file = std::string(CONFIG_PATH) + "CS_UserManual_Robot_State_Message.txt";

    if(!robotInterface->loadConfigure(cfg_file))
    {
        std::cerr << "Failed to load configuration file" << std::endl;
        return -1;
    }
    if(!robotInterface->connect(ip, 30001))
    {
        std::cerr << "Failed to connect 30001 port" << std::endl;
        return -1;
    }

    // 循环获取状态并求逆解
    while(1)
    {
        // 1. 获取当前关节角度
        std::vector<double> currentJoint = robotInterface->getJointActualPos();

        // 2. 通过FK获取位姿
        std::vector<double> currentPose(6, 0.0);
        if(!robot40011->getFKPose(currentJoint, currentPose))
        {
            std::cerr << "Failed to get FK pose" << std::endl;
            continue;
        }

        // 3. 对位姿求逆解
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<double> ikDegrees(6, 0.0);
        bool ikSuccess = robot40011->getIKDegrees(currentPose, ikDegrees);
        auto end = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(end - start).count();
        double freq = 1.0 / elapsed;

        // 打印信息
        std::cout << "Current Joint: ";
        for(auto j : currentJoint) std::cout << j << "\t";
        std::cout << "\nFK Pose: ";
        for(auto p : currentPose) std::cout << p << "\t";
        if(ikSuccess)
        {
            std::cout << "\nIK Degrees: ";
            for(auto d : ikDegrees) std::cout << d << "\t";
            std::cout << "\nIK Time: " << elapsed << " s, Frequency: " << freq << " Hz";
        }
        else
        {
            std::cout << "\nIK solution not found";
        }
        std::cout << std::endl << "----------------------------------------" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz读取
    }

    robotInterface->disconnect();
    robot40011->disconnect();
    return 0;
}
