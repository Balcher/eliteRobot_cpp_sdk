#ifndef __ROBOT_INTERFACE_HPP__
#define __ROBOT_INTERFACE_HPP__

#include "RobotState.hpp"

#include <stdint.h>
#include <string>
#include <vector>

/// @brief 机器人接口类，继承自RobotState
class RobotInterface : public RobotState {
   public:

    /// @brief 控制机器人运动
    /// @param angles 六个轴的旋转角度(弧度制)
    /// @param acceleration 加速度
    /// @param velocity 速度
    /// @param time 时间参数(使用测试数字小就快)
    /// @param r 
    void robotMove(const std::vector<double>& angles, double acceleration,
        double velocity, double time, double r);

    /// @brief 获取时间戳
    /// @return 当前时间戳
    uint64_t getTimeStamp();
    
    /// /// @brief 检查机器人是否通电
    /// @return 如果机器人通电返回 true,否则返回false
    bool isRobotPowerOn();

    /// @brief 检查机器人是否处于紧急停止状态
    /// @return 如果机器人紧急停止返回true，否则返回false
    bool isEmergencyStopped();

    /// @brief 检查机器人是否处于保护停止状态
    /// @return 如果机器人保护停止返回true，否则返回false
    bool isRobotProtectiveStopped();

    /// @brief 检查程序是否正在运行
    /// @return 如果程序正在运行返回true，否则返回false
    bool isProgramRunning();

    /// @brief 检查程序是否处于暂停状态
    /// @return 如果程序处于暂停状态返回 true，否则返回 false
    bool isProgramPaused();

    /// @brief 机器人模式枚举
    enum class RobotMode : uint8_t {
        ROBOT_MODE_DISCONNECTED = 0,          ///< 机器人断开连接
        ROBOT_MODE_CONFIRM_SAFETY = 1,        ///< 正在确认安全状态
        ROBOT_MODE_BOOTING = 2,               ///< 机器人启动中
        ROBOT_MODE_POWER_OFF = 3,             ///< 机器人已关机
        ROBOT_MODE_POWER_ON = 4,              ///< 机器人已通电
        ROBOT_MODE_IDLE = 5,                  ///< 机器人处于空闲状态
        ROBOT_MODE_BACKDRIVE = 6,             ///< 机器人处于反向驱动模式
        ROBOT_MODE_RUNNING = 7,               ///< 机器人正在运行程序
        ROBOT_MODE_UPDATING_FIRMWARE = 8,     ///< 机器人正在更新固件
        ROBOT_MODE_WAITING_CALIBRATION = 9    ///< 机器人正在等待校准
    };
    RobotMode getRobotMode();

    /// @brief 机器人控制模式枚举
    enum class RobotControlMode : uint8_t {
        CONTROL_MODE_POSITION = 0,            ///< 位置控制模式
        CONTROL_MODE_TORQUE = 1               ///< 力矩控制模式

    };

    /// @brief 获取当前机器人控制模式
    /// @return 当前机器人控制模式，返回类型为 RobotControlMode
    RobotControlMode getRobotControlMode();

    /// @brief 获取目标速度比例
    /// @return 目标速度的比例值(0.0到1.0)
    double getTargetSpeedFraction();

    /// @brief 获取速度缩放因子
    /// @return 当前速度缩放因子
    double getSpeedScaling();

    /// @brief 获取目标速度限制
    /// @return 目标速度的限制值
    double getTargetSpeedFractionLimit();

    /// @brief 机器人速度模式枚举
    enum class RobotSpeedMode : uint8_t {
        UNRESTRICTED = 0,                       ///< 无限制速度模式
        MANUAL_HIGH_SPEED = 1,                  ///< 手动高速度模式
        MANUAL_REDUCED_SPEED = 2                ///< 手动降低速度模式

    };

    /// @brief 获取当前机器人速度模式
    /// @return 当前机器人速度模式，返回类型为 RobotSpeedMode
    RobotSpeedMode getRobotSpeedMode();

    /// @brief 检查机器人系统是否处于报警状态
    /// @return 如果机器人系统报警返回true，否则返回false
    bool isRobotSystemInAlarm();
    
    /// @brief 检查机器人是否处于打包模式
    /// @return 如果机器人处于打包模式返回true，否则返回false
    bool isInPackageMode();

    // 获取关节相关状态信息
    std::vector<double> getJointActualPos();              ///< 获取关节实际位置
    std::vector<double> getJointTargetPos();              ///< 获取关节目标位置
    std::vector<double> getJointActualVelocity();         ///< 获取关节实际速度
    std::vector<int32_t> getJointTargetPluse();           ///< 获取关节目标脉冲数
    std::vector<int32_t> getJointActualPluse();           ///< 获取关节实际脉冲数
    std::vector<int32_t> getJointZeroPluse();             ///< 获取关节零点脉冲数
    std::vector<float> getJointCurrent();                 ///< 获取关节电流
    std::vector<float> getJointVoltage();                 ///< 获取关节电压
    std::vector<float> getJointTemperature();             ///< 获取关节温度
    std::vector<float> getJointTorques();                 ///< 获取关节扭矩

    /// @brief 枚举定义：关节模式（joint mode）
    enum class JointMode : uint8_t {
        MODE_RESET = 235,                                 ///< 复位模式
        MODE_SHUTTING_DOWN = 236,                         ///< 关机模式
        MODE_BACKDRIVE = 238,                             ///< 反驱动模式
        MODE_POWER_OFF = 239,                             ///< 断电模式
        MODE_READY_FOR_POWEROFF = 240,                    ///< 准备断电模式
        MODE_NOT_RESPONDING = 245,                        ///< 不响应模式
        MODE_MOTOR_INITIALISATION = 246,                  ///< 电机初始化模式
        MODE_BOOTING = 247,                               ///< 启动模式
        MODE_BOOTLOADER = 249,                            ///< 引导程序模式
        MODE_VIOLATION = 251,                             ///< 违规模式
        MODE_FAULT = 252,                                 ///< 故障模式
        MODE_RUNNING = 253,                               ///< 运行模式
        MODE_IDLE = 255                                   ///< 空闲模式

    };
    std::vector<JointMode> getJointMode();                ///< 获取关节工作模式

    std::vector<double> getTcpPosition();                 ///< 获取TCP位置
    std::vector<double> getTcpOffset();                   ///< 获取TCP偏移量

    std::vector<double> getLimitMinJoint();               ///< 获取关节最小限制
    std::vector<double> getLimitMaxJoint();               ///< 获取关节最大限制
    std::vector<double> getMaxVelocityJoint();            ///< 获取关节最大速度
    std::vector<double> getMaxAccJoint();                 ///< 获取关节最大加速度
    double getDefaultVelocityJoint();                     ///< 获取默认关节速度
    double getDefaultAccJoint();                          ///< 获取默认关节加速度
    double getDefaultToolVelocity();                      ///< 获取默认工具速度
    double getDefaultToolAcc();                           ///< 获取默认工具加速度
    double getEqRadius();                                 ///< 获取等效半径
    std::vector<double> getDhAJoint();                    ///< 获取DH参数 a_i
    std::vector<double> getDhDJoint();                    ///< 获取DH参数 d_i
    std::vector<double> getDhAlphaJoint();                ///< 获取DH参数 \alpha_i
    
    // 获取硬件版本与类型信息
    uint32_t getBoardVersion();                           ///< 获取板卡版本
    uint32_t getControlBoxType();                         ///< 获取机器人类型

    /// @brief 枚举类型：机器人类型
    enum class RobotType : uint32_t {
        ROBOT_TYPE_6203 = 6203,                          ///< 机器人型号 6203
        ROBOT_TYPE_6206 = 6206,                          ///< 机器人型号 6206
        ROBOT_TYPE_6212 = 6212                           ///< 机器人型号 6212

    };
    RobotType getRobotType();

    /// @brief 枚举定义：机器人结构
    enum class RobotStruct : uint32_t {
        ROBOT_STRUCTURE_TYPE_60 = 60,                    ///< 结构类型 60
        ROBOT_STRUCTURE_TYPE_62 = 62,                    ///< 结构类型 62
        ROBOT_STRUCTURE_TYPE_70 = 70                     ///< 结构类型 70

    };
    RobotStruct getRobotStruct();

    std::vector<bool> getStandardDigitalInput();                        ///< 获取标准数字输入
    std::vector<bool> getStandardDigitalOutput();                       ///< 获取标准数字输出
    std::vector<bool> getConfigureDigitalInput();                       ///< 获取配置数字输入
    std::vector<bool> getConfigureDigitalOutput();                      ///< 获取配置数字输出
    std::vector<bool> getToolDigitalInput();                            ///< 获取工具数字输入
    std::vector<bool> getToolDigitalOutput();                           ///< 获取工具数字输出
    
    enum class AnalogMode : uint8_t { CURRENT = 0, VOLTAGE = 1 };       ///< 
    
    std::vector<AnalogMode> getStandardAnalogOutputMode();              ///< 获取标准模拟输出模式
    std::vector<AnalogMode> getStandardAnalogInputMode();               ///< 获取标准模拟输入模式
    AnalogMode getToolAnalogOutputMode();                               ///< 获取工具模拟输出模式
    AnalogMode getToolAnalogInputMode();                                ///< 获取工具模拟输入模式
    std::vector<double> getStandardAnalogOutputValue();                 ///< 获取标准模拟输出值
    std::vector<double> getStandardAnalogInputValue();                  ///< 获取标准模拟输入值
    double getToolAnalogOutputValue();                                  ///< 获取工具模拟输出值
    double getToolAnalogInputValue();                                   ///< 获取工具模拟输入值
    
    // 获取机器人电气参数
    float getBoardTemperature();                                        ///< 获取板卡温度
    float getRobotVoltage();                                            ///< 获取机器人电压
    float getRobotCurrent();                                            ///< 获取机器人电流
    float getIOCurrent();                                               ///< 获取IO电流

    // 枚举定义：安全模式
    enum class SafetyMode : uint8_t {
        SAFETY_MODE_NORMAL = 1,                                         ///< 正常模式
        SAFETY_MODE_REDUCED = 2,                                        ///< 降低模式
        SAFETY_MODE_PROTECTIVE_STOP = 3,                                ///< 保护停止模式
        SAFETY_MODE_RECOVERY = 4,                                       ///< 恢复模式
        SAFETY_MODE_SAFEGUARD_STOP = 5,                                 ///< 安全停止模式
        SAFETY_MODE_SYSTEM_EMERGENCY_STOP = 6,                          ///< 系统紧急停止模式
        SAFETY_MODE_ROBOT_EMERGENCY_STOP = 7,                           ///< 机器人紧急停止模式
        SAFETY_MODE_VIOLATION = 8,                                      ///< 违规模式
        SAFETY_MODE_FAULT = 9,                                          ///< 故障模式
        SAFETY_MODE_VALIDATE_JOINT_ID = 10,                             ///< 验证关节ID
        SAFETY_MODE_UNDEFINED_SAFETY_MODE = 11,                         ///< 未定义的安全模式
        SAFETY_MODE_AUTOMATIC_MODE_SAFEGUARD_STOP = 12,                 ///< 自动模式安全停止
        SAFETY_MODE_SYSTEM_THREE_POSITION_ENABLING_STOP = 13            ///< 系统三位置启用停止
    };
    
    // 获取安全模式
    SafetyMode getBordSafeMode();                                       ///< 获取主板安全模式
    bool isRobotInReducedMode();                                        ///< 判断机器人是否处于降低模式
    bool getOperationalModeSelectorInput();                             ///< 获取操作模式选择器输入
    bool getThreepositionEnablingDeviceInput();                         ///< 获取三位置启动设备输入
    SafetyMode getMasterboardSafetyMode();                              ///< 获取主板安全模式

    // 获取自由驱动相关状态
    bool isFreedriveButtonPressed();                                    ///< 判断自由驱动按钮是否按下
    bool isFreedriveIOEnabled();                                        ///< 判断自由驱动IO是否启用
    bool isDynamicCollisionDetectEnabled();                             ///< 判断动态碰撞检测是否启用

    // 获取工具相关的电气参数
    float getToolVoltage();                                             ///< 获取工具的电压值
    float getToolCurrent();                                             ///< 获取工具的电流值

    // 枚举定义：工具输出电压等级
    enum class ToolOutputVoltage : uint8_t { 
        VOLTAGE_0_LEVEL = 0,                                            ///< 工具输出电压为 0v
        VOLTAGE_12_LEVEL = 12,                                          ///< 工具输出电压为 12v
        VOLTAGE_24_LEVEL = 24                                           ///< 工具输出电压为 24v
    };

    // 获取工具电气状态
    ToolOutputVoltage getToolOutputVoltage();                           ///< 获取工具的输出电压等级
    float getToolTemperature();                                         ///< 获取工具的温度

    // 定义关节模式（JointMode）作为工具模式(ToolMode)的别名
    typedef JointMode ToolMode;                                         
    ToolMode getToolMode();                                             ///< 返回工具的当前模式

    // 获取安全相关的信息
    uint32_t getSafetyCRCNum();                                         ///< 获取安全相关的CRC校验码
    
    // 枚举定义：安全操作模式
    enum class SafeOptMode : int8_t { 
        NONE = -1,                                                      ///< 无安全操作模式
        AUTOMATIC = 0,                                                  ///< 自动安全模式
        MANUAL = 1                                                      ///< 手动安全模式
    };
    
    // 获取当前的安全操作模式
    SafeOptMode getSafetyOperationalMode();                             ///< 返回机器人当前的安全操作模式

    // 获取肘部相关参数
    std::vector<double> getCurrentElbowPos();                           ///< 获取当前肘部位置的坐标(可能是 3D 坐标)
    double getElbowRadius();                                            ///< 获取肘部的半径信息

    // 工具 RS485 接口相关功能
    bool isToolRS485Enable();                                           ///< 判断工具的 RS485 接口是否启用
    uint32_t getToolRS485Baudrate();                                    ///< 获取工具 RS485 的波特率
    uint32_t getToolRS485Parity();                                      ///< 获取工具 RS485 的校验位设置
    uint32_t getToolRS485Stopbits();                                    ///< 获取工具 RS485 的停止位设置
    bool isToolRS485ModbusMode();                                       ///< 判断工具是否处于 Modbus 模式

    // 枚举定义：工具 RS485 的使用模式
    enum class ToolRS485Usage : uint8_t { 
        SCRIPT_MODE,                                                    ///< RS485 脚本
        DAEMON_MODE                                                     ///< RS485 守护进程模式
    };
    ToolRS485Usage getToolRS485Usage();
    
    // 模板函数：从状态列表中获取指定类型的状态值
    template <typename T>
    std::vector<T> getStatesVector(const char* state_list[], int list_size) {
        std::vector<T> result;
        // 遍历状态列表，调用 getItemData 方法获取每个状态值并存储到结果向量中
        for (int i = 0; i < list_size; i++) {
            result.push_back(getItemData<T>(state_list[i], nullptr));
        }
        return result;
    }

    RobotInterface();
    ~RobotInterface();
};

#endif