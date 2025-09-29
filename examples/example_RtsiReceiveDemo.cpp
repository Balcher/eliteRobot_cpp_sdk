#include <iostream>
#include "rtsiReceive.h"
#include <memory>
#include <chrono>
#include <iomanip> // 用于设置输出精度

int main()
{
    std::shared_ptr<RtsiReceive> rtsi_ptr = std::make_shared<RtsiReceive>("192.168.205.133", 20);

    // 测试一个初始关节位置得到TCP
    std::vector<double> joint_positions = {0.0, -1.57, 0.0, -1.57, 1.57, 0.0};
    std::vector<double> tcp_positions = rtsi_ptr->getTcpPosition(joint_positions);
    if (tcp_positions.size() == 6)
    {
        for (double p : tcp_positions) std::cout << p << "\t";
        std::cout << std::endl;
    }

    rtsi_ptr->RtsiStart();

    try
    {
        while (true)
        {
            auto start = std::chrono::high_resolution_clock::now();

            // 获取关节角度
            std::vector<double> currentPositions = rtsi_ptr->GetActualJointPositions();
            if (currentPositions.size() == 6)
            {
                // 获取TCP位姿
                std::vector<double> tcp_positions = rtsi_ptr->getTcpPosition(currentPositions);

                // 打印一行数据
                std::cout << std::fixed << std::setprecision(6);
                std::cout << "Joints: ";
                for (double j : currentPositions) std::cout << j << " ";
                std::cout << "| TCP: ";
                for (double t : tcp_positions) std::cout << t << " ";
                
                auto end = std::chrono::high_resolution_clock::now();
                double elapsed = std::chrono::duration<double>(end - start).count();
                double freq = (elapsed > 0.0) ? 1.0 / elapsed : 0.0;

                std::cout << "| Time: " << elapsed << " s, Freq: " << freq << " Hz" << std::endl;
            }

            // 可控制读取频率
            std::this_thread::sleep_for(std::chrono::milliseconds(4)); // 大约250Hz
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception caught: " << e.what() << std::endl;
    }

    rtsi_ptr->RtsiStop();

    return 0;
}
