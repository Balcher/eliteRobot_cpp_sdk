#pragma once

#include "Rtsi.hpp"    // RTSI接口头文件
#include <iostream>    // 标准输入输出流
#include <memory>      // 智能指针
#include <thread>      // 线程库
#include <chrono>      // 时间库
#include <Eigen/Dense> // Eigen库，用于处理向量和矩阵

struct DHParameters
{
    double d;     // 连杆偏移
    double a;     // 连杆长度
    double alpha; // 连杆扭转角
};

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

    /**
     * @brief 设置TCP偏移
     *
     * @param x TCP在X轴上的偏移量
     * @param y TCP在Y轴上的偏移量
     * @param z TCP在Z轴上的偏移量
     * @param roll TCP绕X轴的旋转角度
     * @param pitch TCP绕Y轴的旋转角度
     * @param yaw TCP绕Z轴的旋转角度
     */
    void setTCPOffset(double x, double y, double z, double roll, double pitch, double yaw);

    /** 
     * @brief Get the Tcp Position object（只能TCP不带偏移情况）
     *
     * @param joint_positions
     * @return std::vector<double> (m,rad)
     */
    std::vector<double> getTcpPosition(const std::vector<double> &joint_positions);

private:
    std::unique_ptr<Rtsi> rt;        // RTSI连接对象
    Rtsi::DataRecipePtr out_recipe1; // 订阅负载质量和重心位置
    Rtsi::DataRecipePtr out_recipe2; // 订阅时间戳和实际关节位置

    double tcp_offset_x, tcp_offset_y, tcp_offset_z;          // TCP偏移量
    double tcp_offset_roll, tcp_offset_pitch, tcp_offset_yaw; // TCP偏移角度

    // 固定的DH参数
    std::vector<DHParameters> dh_parameters = {
        {0.235, 0.0, 0.0},     // d1, a1, alpha1
        {0.0, 0.0, M_PI_2},    // d2, a2, alpha2
        {0.0, -0.9, 0.0},      // d3, a3, alpha3
        {0.1725, -0.772, 0.0}, // d4, a4, alpha4
        {0.128, 0.0, M_PI_2},  // d5, a5, alpha5
        {0.125, 0.0, -M_PI_2}, // d6, a6, alpha6
    };

    /**
     * @brief 计算DH参数的变换矩阵
     *
     * @param dh_params DH参数
     * @param theta     关节角度(弧度)
     * @return Eigen::Matrix4d 变换矩阵
     */
    Eigen::Matrix4d computeTransformationMatrix(const DHParameters &dh_params, double theta);

    /**
     * @brief 计算正向运动学(不包括TCP偏移)
     *
     * @param joint_positions 6个关节的角度
     * @return Eigen::Matrix4d
     */
    Eigen::Matrix4d forwardKinematics(const std::vector<double> &joint_positions);

    // TODO: 有TCP偏移情况，这个正向运动学怎么解决
    /**
     * @brief Get the Tcp Transformation Matrix object
     *
     * @param joint_positions
     * @return Eigen::Matrix4d
     */
    Eigen::Matrix4d getTcpTransformationMatrix(const std::vector<double> &joint_positions);

};