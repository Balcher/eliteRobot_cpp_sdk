#include <iostream>
#include "rtsiReceive.h"
#include <memory>

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
    while (1)
    {
        std::vector<double> joint_positions = rtsi_ptr->GetActualJointPositions();
        if (joint_positions.size() == 6)
        {
            // for (size_t i = 0; i < 6; i++)
            // {
            //     std::cout << joint_positions[i] << "\t";
            // }
            // std::cout << "------------------------------" << std::endl;

            std::vector<double> tcp_positions = rtsi_ptr->getTcpPosition(joint_positions);
            if (tcp_positions.size() == 6)
            {
                for (size_t i = 0; i < 6; i++)
                {
                    std::cout << tcp_positions[i] << "\t";
                }
                std::cout << std::endl;
            }
        }
    }
}