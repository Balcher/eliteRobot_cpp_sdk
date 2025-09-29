#include "Rtsi.hpp" 
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <iomanip> // 设置输出精度

int main()
{
    std::string ip = "192.168.205.133"; // 固定IP

    auto rt = std::make_unique<Rtsi>();
    if(!rt->connect(ip))
    {
        std::cout << "Connect fail" << std::endl;
        return -1;
    }
    if(!rt->versionCheck())
    {
        std::cout << "Version check fail" << std::endl;
        return -1;
    }

    auto out_recipe1 = rt->outputSubscribe("payload_mass,payload_cog", 250);
    auto out_recipe2 = rt->outputSubscribe("timestamp,actual_joint_positions", 125);
    auto input_recipe = rt->inputSubscribe("speed_slider_mask,speed_slider_fraction");

    rt->sendTextMessage("New RTSI Connect", "RTSI CPP Client", Rtsi::MessageType::INFO_MESSAGE);
    rt->start();

    double speed_slider_fraction = 0;
    while(1)
    {
        auto start = std::chrono::high_resolution_clock::now();
        auto &recipe = rt->getOutputDataToRecipe();

        std::cout << std::fixed << std::setprecision(6);

        // 输出数据行
        if(recipe->getID() == out_recipe2->getID())
        {
            std::cout << "Timestamp: " << (*recipe)["timestamp"].value.d_64 << std::endl << " Joints: ";
            for(int i = 0; i < 6; ++i)
                std::cout << (*recipe)["actual_joint_positions"].value.v6d[i] << " ";
        }
        else if(recipe->getID() == out_recipe1->getID())
        {
            std::cout << "Timestamp: 0.000000 " << std::endl << " Payload: mass=" 
                      << (*recipe)["payload_mass"].value.d_64 << " COG=";
            for(int i = 0; i < 3; ++i)
                std::cout << (*recipe)["payload_cog"].value.v3d[i] << " ";
        }
        std::cout << std::endl;

        // 输出耗时行
        auto end = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(end - start).count();
        double freq = (elapsed > 0.0) ? 1.0 / elapsed : 0.0;
        std::cout << "Time: " << elapsed << " s, Freq: " << freq << " Hz" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }

    rt->pause();
    rt->disconnect();

    return 0;
}
