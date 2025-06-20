/**
 * @note：
 * 关节角度设置了一个线程来获取，线程的延迟时间是更新时间的1/10,
 * 关节角度有变化情况设置了回调函数用来获取角度的更新
 * 负载质量和重心位置未做处理
 *
 * 获取关节角度的接口：
 * 1. 同步获取接口，（GetActualJointPositions）能够实时获取关节角度。
 * 2. 异步获取接口，通过回调函数（setEventDrivenJointUpdataCallback）来异步获取角度，事件驱动的，关节角度有变化才会更新
 * 3. 异步获取接口，通过回调函数(setRealTimeJointUpdataCallback)，异步实时获取角度
 *
 */
#pragma once

#include "Rtsi.hpp"    // RTSI接口头文件
#include <iostream>    // 标准输入输出流
#include <memory>      // 智能指针
#include <thread>      // 线程库
#include <chrono>      // 时间库
#include <Eigen/Dense> // Eigen库，用于处理向量和矩阵
#include <functional>  // 用于std::function
#include <mutex>       // 互斥锁
struct DHParameters
{
    double d;     // 连杆偏移
    double a;     // 连杆长度
    double alpha; // 连杆扭转角
};

// 固定的DH参数
static std::vector<DHParameters> dh_parameters = {
    {0.235, 0.0, 0.0},     // d1, a1, alpha1
    {0.0, 0.0, M_PI_2},    // d2, a2, alpha2
    {0.0, -0.9, 0.0},      // d3, a3, alpha3
    {0.1725, -0.772, 0.0}, // d4, a4, alpha4
    {0.128, 0.0, M_PI_2},  // d5, a5, alpha5
    {0.125, 0.0, -M_PI_2}, // d6, a6, alpha6
};

class RtsiReceive
{
public:
    // 定义回调函数类型
    using JointUpdataCallback = std::function<void(const std::vector<double> &)>;

    RtsiReceive() = default;
    explicit RtsiReceive(const std::string &ip, int jointRate = 100, int payloadRate = 100);
    ~RtsiReceive();

    void RtsiStart();
    void RtsiStop();
    std::vector<double> GetActualJointPositions(); // 同步获取当前关节角
    std::vector<double> GetPayloadCog();           // 负载质量和重心位置 // TODO： 能否正常使用未知

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
     * @brief Get the Tcp Position object
     *
     * @param joint_positions
     * @return std::vector<double> (m,rad)
     */
    std::vector<double> getTcpPosition(const std::vector<double> &joint_positions);

    /**
     * @brief 设置关节更新回调函数（异步获取关节角度，关节变化才会触发）
     *
     * @param callback 回调函数
     */
    void setEventDrivenJointUpdataCallback(JointUpdataCallback callback);

    /**
     * @brief 设置实时的关节更新回调函数
     *
     * @param callback 回调函数
     */
    void setRealTimeJointUpdataCallback(JointUpdataCallback callback);

    /**
     * @brief 获取机械臂上面各关节位姿
     *
     * @param degrees 关节角度
     * @return std::vector<std::vector<double>> 6*6矩阵，分别表示基座、肩部、肘部、手腕1、手腕2、手腕3的位姿
     */
    static std::vector<std::vector<double>> getJointActualPosition(std::vector<double> degrees);

private:
    std::unique_ptr<Rtsi> rt;                   // RTSI连接对象
    Rtsi::DataRecipePtr out_recipe1;            // 订阅负载质量和重心位置
    Rtsi::DataRecipePtr out_recipe2;            // 订阅时间戳和实际关节位置
    std::vector<double> m_last_joint_positions; // 缓存上一次的数据
    int m_jointRate;                            // 关节获取速率

    std::thread m_update_thread; // 子线程
    bool m_running = false;      // 线程运行标志
    std::mutex m_mutex;          // 互斥锁

    double tcp_offset_x, tcp_offset_y, tcp_offset_z;          // TCP偏移量
    double tcp_offset_roll, tcp_offset_pitch, tcp_offset_yaw; // TCP偏移角度

    // 回调函数
    JointUpdataCallback event_driven_joint_update_callback;
    JointUpdataCallback real_time_joint_update_callback;

    /**
     * @brief 计算DH参数的变换矩阵
     *
     * @param dh_params DH参数
     * @param theta     关节角度(弧度)
     * @return Eigen::Matrix4d 变换矩阵
     */
    static Eigen::Matrix4d computeTransformationMatrix(const DHParameters &dh_params, double theta);

    /**
     * @brief 计算正向运动学(不包括TCP偏移)
     *
     * @param joint_positions 6个关节的角度
     * @return Eigen::Matrix4d
     */
    Eigen::Matrix4d forwardKinematics(const std::vector<double> &joint_positions);

    /**
     * @brief 子线程更新关节角度并触发回调
     *
     */
    void updateJointPosition();

    /**
     * @brief 从Matrix4d形式解算成 xyz,rpy
     *
     * @param matrix
     * @return std::vector<double>
     */
    static std::vector<double> matrixToVector(const Eigen::Matrix4d &matrix);
};