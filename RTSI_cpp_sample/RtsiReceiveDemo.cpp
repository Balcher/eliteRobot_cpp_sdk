#include <iostream>
#include "rtsiReceive.h"
#include <memory>

// 回调函数实现
void onJointUpdate(const std::vector<double> &jointPositions)
{
    std::cout << "Joint positions updated: ";
    for (double pos : jointPositions)
    {
        std::cout << pos << " ";
    }
    std::cout << std::endl;
}

int main()
{
    std::shared_ptr<RtsiReceive> rtsi_ptr;
    rtsi_ptr = std::make_shared<RtsiReceive>("192.168.205.133", 20);

    std::vector<double> joint_positions = {0.0, -1.57, 0.0, -1.57, 1.57, 0.0};
    std::vector<double> tcp_positions = rtsi_ptr->getTcpPosition(joint_positions);
    if (tcp_positions.size() == 6)
    {
        for (size_t i = 0; i < 6; i++)
        {
            std::cout << tcp_positions[i] << "\t";
        }
        std::cout << std::endl;
    }

    rtsi_ptr->RtsiStart();

    // 模拟程序运行一段时间
    try
    {
        while (true)
        {
            // 可以在这里添加其他业务逻辑
            // 例如，主动获取关节角度
            std::vector<double> currentPositions = rtsi_ptr->GetActualJointPositions();
            if (currentPositions.size() == 6)
            {

                std::vector<double> tcp_positions = rtsi_ptr->getTcpPosition(currentPositions);

                for (double pos : tcp_positions)
                {
                    std::cout << pos << " ";
                }
                std::cout << std::endl;
            }

            // 线程休眠 1 秒
            std::this_thread::sleep_for(std::chrono::seconds(1));

            static int i = 0;
            i++;
            if (i > 10)
            {
                break;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception caught: " << e.what() << std::endl;
    }

    // 停止 RTSI 连接和更新线程
    rtsi_ptr->RtsiStop();
}