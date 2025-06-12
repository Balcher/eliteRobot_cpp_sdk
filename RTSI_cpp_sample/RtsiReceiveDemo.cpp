#include <iostream>
#include "rtsiReceive.h"

int main()
{
    RtsiReceive rtsi("192.168.205.133", 20);
    rtsi.RtsiStart();
    while (1)
    {
        std::vector<double> joint_positions = rtsi.GetActualJointPositions();
        if (joint_positions.size() == 6)
        {
            for (size_t i = 0; i < 6; i++)
            {
                std::cout << joint_positions[i] << "\t";
            }
            std::cout << std::endl;
        }
    }
}