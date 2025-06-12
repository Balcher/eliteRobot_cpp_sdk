#include "Rtsi.hpp" // RTSI接口头文件
#include <iostream> // 标准输入输出流
#include <memory>   // 智能指针
#include <thread>   // 线程库
#include <chrono>   // 时间库

int main(int argv, char **argc)
{
    // 参数检查
    if (argv < 2)
    {
        std::cout << "Must input IP address" << std::endl;
        return -1;
    }

    // 创建RTSI连接对象
    std::unique_ptr<Rtsi> rt = std::make_unique<Rtsi>();
    if (!rt->connect(argc[1]))
    {
        std::cout << "Connect fail" << std::endl;
        return -1;
    }

    // 版本检查
    if (!rt->versionCheck())
    {
        std::cout << "Version check fail" << std::endl;
        return -1;
    }

    // 订阅数据
    // 订阅两个输出数据和一个输入数据
    Rtsi::DataRecipePtr out_recipe1 = rt->outputSubscribe("payload_mass,payload_cog", 250);           // 订阅负载质量和重心位置
    Rtsi::DataRecipePtr out_recipe2 = rt->outputSubscribe("timestamp,actual_joint_positions", 125);   // 订阅时间戳和实际关节位置
    Rtsi::DataRecipePtr input_recipe = rt->inputSubscribe("speed_slider_mask,speed_slider_fraction"); // 订阅速度滑块掩码和速度滑块分数
    rt->sendTextMessage("New RTSI Connect", "RTSI CPP Client", Rtsi::MessageType::INFO_MESSAGE);      // 发送文本消息
    rt->start();                                                                                      // 启动数据流
    double speed_slider_fraction = 0;
    while (speed_slider_fraction < 1)
    {
        // 设置输入数据
        // (*input_recipe)["speed_slider_mask"].value.u_32 = 1;
        // (*input_recipe)["speed_slider_fraction"].value.d_64 = speed_slider_fraction;

        // 发送输入数据
        // rt->sendInputData(input_recipe);

        // 获取输出数据
        Rtsi::DataRecipePtr &recipe = rt->getOutputDataToRecipe();

        // 处理不同类型的输出数据
        if (recipe->getID() == out_recipe1->getID())
        {
            // 处理有效的负载数据
            std::cout << "Recipe 1:" << std::endl;
            std::cout << "\tpayload_mass: " << (*recipe)["payload_mass"].value.d_64 << std::endl;
            std::cout << "\tpayload_cog: ";
            for (size_t i = 0; i < 3; i++)
            {
                std::cout << (*recipe)["payload_cog"].value.v3d[i] << "\t";
            }
            std::cout << std::endl;
        }
        else if (recipe->getID() == out_recipe2->getID())
        {
            // 处理关节位置数据...
            std::cout << "Recipe 2:" << std::endl;
            std::cout << "\ttimestamp: " << (*recipe)["timestamp"].value.d_64 << std::endl;
            std::cout << "\tpayload_cog: ";
            for (size_t i = 0; i < 6; i++)
            {
                std::cout << (*recipe)["actual_joint_positions"].value.v6d[i] << "\t";
            }
            std::cout << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
        speed_slider_fraction += 0.001;
    }
    rt->pause();
    rt->disconnect();

    return 0;
}