#pragma once

#include "Rtsi.hpp" // RTSI接口头文件
#include <iostream> // 标准输入输出流
#include <memory>   // 智能指针
#include <thread>   // 线程库
#include <chrono>   // 时间库

class RtsiReceive
{
public:
    RtsiReceive() = default;
    explicit RtsiReceive(const std::string &ip, int jointRate = 100, int payloadRate = 100);
    ~RtsiReceive();

    void RtsiStart();
    void RtsiStop();
    std::vector<double> GetActualJointPositions();
    std::vector<double> GetPayloadCog();


private:
    std::unique_ptr<Rtsi> rt; // RTSI连接对象
    Rtsi::DataRecipePtr out_recipe1; // 订阅负载质量和重心位置
    Rtsi::DataRecipePtr out_recipe2; // 订阅时间戳和实际关节位置

};