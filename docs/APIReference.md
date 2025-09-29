## RobotState
- 描述：解析机器人TCP 30001端口数据报文。

### bool loadConfigure(const std::string& file_path)
- 描述：加载配置文件
- 输入参数：
  - file_path：配置文件路径
- 返回值：成功为true，失败为false。
- 注：
  - 配置文件为文本文件，获取方式为：打开英文版的30001报文说明Excel --> 选择需要版本的sheet --> 另存为.txt文件-->打开txt文件，删除非RobotState部分的报文描述。。
  - 至少使用的控制器版本和配置文件的版本要能匹配，否则会无法解析报文内容。


### bool connect(const std::string& ip, int port)
- 描述：连接机器人
- 输入参数：
  - ip：机器人IP
  - port：TCP端口号，即30001
- 返回值：成功为true，失败为false。


### bool isConnect()
- 描述：是否连接到机器人
- 返回值：连接为true，未连接为false。


### void disconnect()
- 描述：断开与机器人的链接

### bool findItem(const std::string& name, Item* item)
- 描述：寻找数据项
- 参数：
  - name：数据项名称
  - item:数据项
- 返回值：成功为true，失败为false。
- 注：数据项名称由英文版的30001报文说明Excel得到，基本规则为：子报文名称_项目名称。例如，打开表格后能看到“Robot mode sub-package”，即机器人模式子报文，其中数据项“timestamp”在存储在本类中的名字为“Robot_mode_timestamp”

### template<typename T> T getItemData(const std::string& name, T* data)
- 描述：获取数据项数据
- 参数：
  - name：数据项名称
  - data：获取到的数据输出，可以为nullptr
- 返回值：获取到的数据

### void setExceptionCallback(const std::function<void(const RobotException&)>& callback)
- 描述：设置当异常发生时的回调函数
- 参数：
  - callback：异常回调函数

## RobotInterface
- 描述：对RobotState进行了公用继承，并基于机器人控制器 Eliserver 2.6.0 版本进行了接口封装。
- 注：
  - 在控制器版本和配置文件版本匹配的前提下，如果不是 2.6.0 版本，部分接口将无效，以及可能会缺少部分数据项接口，可以考虑对该类进行适当的删改。

### uint64_t getTimeStamp()
- 描述：获取机器人时间戳（微秒）
- 返回值：机器人时间戳

### bool isRobotPowerOn()
- 描述：机器人是否上电
- 返回值：true为上电，false为下电

### bool isEmergencyStopped()
- 描述：是否处于急停状态
- 返回值：true为处于急停状态

### bool isRobotProtectiveStopped()
- 描述：是否处于保护停止状态
- 返回值：true为处于保护停止状态

### bool isProgramRunning()
- 描述：程序是否运行
- 返回值：true为程序正在运行。


### bool isProgramPaused()
- 描述：程序是否暂停
- 返回值：true为程序暂停。

### RobotMode getRobotMode()
- 描述：获取机器人模式
- 注：
    | 模式                       | 值   | 意义                     |
    | ---------------------------- | ---- | ------------------------ |
    | ROBOT_MODE_DISCONNECTED      | 0    | 机器人连接断开           |
    | ROBOT_MODE_CONFIRM_SAFETY    | 1    | 确认安全参数             |
    | ROBOT_MODE_BOOTING           | 2    | 机器人上电中             |
    | ROBOT_MODE_POWER_OFF         | 3    | 机器人电源关闭           |
    | ROBOT_MODE_POWER_ON          | 4    | 机器人电源打开           |
    | ROBOT_MODE_IDLE              | 5    | 机器人待机（未释放抱闸） |
    | ROBOT_MODE_BACKDRIVE         | 6    | 反向驱动                 |
    | ROBOT_MODE_RUNNING           | 7    | 机器人运行（已释放抱闸） |
    | ROBOT_MODE_UPDATING_FIRMWARE | 8    | 固件升级中               |



### RobotControlMode getRobotControlMode()
- 描述：获取机器人控制模式
- 注：
    | 模式                       | 值   | 意义      |
    | ------------------------- | ---- | ------ |
    |CONTROL_MODE_POSITION      | 0    | 位置模式   |
    |CONTROL_MODE_TORQUE        | 1    | 力矩模式   |

### double getTargetSpeedFraction()
- 描述：获取机器人目标运行速度百分比

### double getSpeedScaling()
- 描述：获取实际机器人运行速度

### double getTargetSpeedFractionLimit()
- 描述：获取机器人运行目标速度比例限制

### RobotSpeedMode getRobotSpeedMode()
- 描述：获取机器人速度模式
- 注：
    | 模式                       | 值   | 意义      |
    | ------------------------- | ---- | ------ |
    | UNRESTRICTED              | 0     | 未配置 |
    | MANUAL_HIGH_SPEED         | 1     | 手动高速 |
    | MANUAL_REDUCED_SPEED      | 2     | 手动低速 |


### bool isRobotSystemInAlarm()
- 描述：机器人是否处于报警状态

### bool isInPackageMode()
- 描述：机器人是否处于打包模式

### std::vector<double> getJointActualPos()
- 描述：获取关节真实位置

### std::vector<double> getJointTargetPos()
- 描述：获取关节目标位置


### std::vector<double> getJointActualVelocity()
- 描述：获取关节真实速度


### std::vector<int32_t> getJointTargetPluse()
- 描述：获取关节目标脉冲

### std::vector<int32_t> getJointActualPluse()
- 描述：获取关节实际脉冲


### std::vector<int32_t> getJointZeroPluse()
- 描述：获取关节零位脉冲


### std::vector<float> getJointCurrent()
- 描述：获取关节电流


### std::vector<float> getJointVoltage()
- 描述：获取关节电压


### std::vector<float> getJointTemperature()
- 描述：获取关节温度

### std::vector<float> getJointTorques()
- 描述：获取关节力矩

### std::vector<JointMode> getJointMode()
- 描述：获取关节模式
- 注：
    | 模式                       | 值   | 意义      |
    | ------------------------- | ---- | ------ |
    | MODE_RESET                | 235 | 重置                   |
    | MODE_SHUTTING_DOWN        | 236 | 正在关闭               |
    | MODE_BACKDRIVE            | 238 | 反向驱动               |
    | MODE_POWER_OFF            | 239 | 电源关闭               |
    | MODE_READY_FOR_POWEROFF   | 240 | 准备关闭电源           |
    | MODE_NOT_RESPONDING       | 245 | 无响应                 |
    | MODE_MOTOR_INITIALISATION | 246 | 初始化（正在释放抱闸）  |
    | MODE_BOOTING              | 247 | 启动中                 |
    | MODE_BOOTLOADER           | 249 | boolloader模式         |
    | MODE_VIOLATION            | 251 | 违规                   |
    | MODE_FAULT                | 252 | 错误                   |
    | MODE_RUNNING              | 253 | 运行中                 |
    | MODE_IDLE                 | 255 | 待机                   |

### std::vector<double> getTcpPosition()
- 描述：获取TCP位置
- 注：返回值长度为6，分别为[px py pz rx ry rz]

### std::vector<double> getTcpOffset()
- 描述：获取TCP偏移值
- 注：返回值长度为6，分别为[px py pz rx ry rz]

### std::vector<double> getLimitMinJoint()
- 描述：获取关节最小硬极限
  
### std::vector<double> getLimitMaxJoint()
- 描述：获取关节最大硬极限

### std::vector<double> getMaxVelocityJoint()
- 描述：获取最大关节速度

### std::vector<double> getMaxAccJoint()
- 描述：获取最大关节加速度

### double getDefaultVelocityJoint()
- 描述：获取关节默认速度

### double getDefaultAccJoint()
- 描述：获取关节默认加速度

### double getDefaultToolVelocity()
- 描述：获取工具默认速度

### double getDefaultToolAcc()
- 描述：获取工具默认加速度

### double getEqRadius()
- 描述：默认交融半径

### std::vector<double> getDhAJoint()
- 描述：DH参数A

### std::vector<double> getDhDJoint()
- 描述：DH参数D

### std::vector<double> getDhAlphaJoint()
- 描述：DH参数Alpha
    
### uint32_t getBoardVersion()
- 描述：主板版本

### uint32_t getControlBoxType()
- 描述：控制柜类型

### RobotType getRobotType()
- 描述：获取机器人类型
- 注：
    | 模式                       | 值   | 意义      |
    | ------------------------- | ---- | ------ |
    | ROBOT_TYPE_6203           | 6203 | 6203类型 |
    |ROBOT_TYPE_6206            | 6206 | 6206类型 |
    |ROBOT_TYPE_6212            | 6212 | 6212类型 |

### RobotStruct getRobotStruct();
- 描述：获取机器人构型
- 注：
    | 模式                       | 值   | 意义      |
    | ------------------------- | ---- | ------ |
    | ROBOT_STRUCTURE_TYPE_60 | 60 | 60 |
    | ROBOT_STRUCTURE_TYPE_62 | 62 | 62 |
    | ROBOT_STRUCTURE_TYPE_70 | 70 | 70 |


### std::vector<bool> getStandardDigitalInput()
- 描述：获取标准数字输入

### std::vector<bool> getStandardDigitalOutput()
- 描述：获取标准数字IO输出

### std::vector<bool> getConfigureDigitalInput()
- 描述：获取可配置数字输入


### std::vector<bool> getConfigureDigitalOutput()
- 描述：获取可配置输出


### std::vector<bool> getToolDigitalInput()
- 描述：获取工具数字IO输入


### std::vector<bool> getToolDigitalOutput();
- 描述：获取工具数字输出

### std::vector<AnalogMode> getStandardAnalogOutputMode()
- 描述：获取标准模拟输出模式
- 注：
    | 模式                       | 值   | 意义      |
    | ------------------------- | ---- | ------ |
    | CURRENT | 0 | 60 |
    | VOLTAGE | 1 | 62 |

### std::vector<AnalogMode> getStandardAnalogInputMode()
- 描述：获取标准模拟输入
- 注：
    | 模式                       | 值   | 意义      |
    | ------------------------- | ---- | ------ |
    | CURRENT | 0 | 60 |
    | VOLTAGE | 1 | 62 |

### AnalogMode getToolAnalogOutputMode()
- 描述：获取工具模拟输出模式
- 注：
    | 模式 | 值 | 意义 |
    | ------------------------- | ---- | ------ |
    | CURRENT | 0 | 60 |
    | VOLTAGE | 1 | 62 |

### AnalogMode getToolAnalogInputMode()
- 描述：获取工具模拟输入模式
- 注：
    | 模式 | 值  | 意义 |
    | ------------------------- | ---- | ------ |
    | CURRENT | 0 | 60 |
    | VOLTAGE | 1 | 62 |


### std::vector<double> getStandardAnalogOutputValue()
-描述：获取标准模拟输入的值


### std::vector<double> getStandardAnalogInputValue()
-描述：获取标准输出的值

### double getToolAnalogOutputValue()
-描述：获取工具模拟输出值

### double getToolAnalogInputValue()
- 描述：获取工具模拟输入值

### float getBoardTemperature()
- 描述：获取主板温度

### float getRobotVoltage()
- 描述：获取机器人电压

### float getRobotCurrent()
- 描述：获取机器人电流

### float getIOCurrent()
- 描述：获取IO电流

### SafetyMode getBordSafeMode()
- 描述：获取主板安全模式
- 注：
    | 模式                            | 值   | 意义                         |
    | --------------------------------- | ---- | ---------------------------- |
    | SAFETY_MODE_NORMAL                | 1    | 正常模式                     |
    | SAFETY_MODE_REDUCED               | 2    | 缩减模式                     |
    | SAFETY_MODE_PROTECTIVE_STOP       | 3    | 保护性停止                   |
    | SAFETY_MODE_RECOVERY              | 4    | 恢复模式                     |
    | SAFETY_MODE_SAFEGUARD_STOP        | 5    | 安全防护停止                 |
    | SAFETY_MODE_SYSTEM_EMERGENCY_STOP | 6    | 系统急停（安全IO触发）       |
    | SAFETY_MODE_ROBOT_EMERGENCY_STOP  | 7    | 机器人急停（急停按钮触发）   |
    | SAFETY_MODE_VIOLATION             | 8    | 安全违规（超出安全参数限制） |
    | SAFETY_MODE_FAULT                 | 9    | 安全错误                     |
    | SAFETY_MODE_VALIDATE_JOINT_ID     | 10   | 关节违规                     |
    | SAFETY_MODE_UNDEFINED_SAFETY_MODE | 11   | 未知安全模式                 |

### bool isRobotInReducedMode()
- 描述：是否处于缩减模式

### bool getOperationalModeSelectorInput()
- 描述：模式选择器是否打开

### bool getThreepositionEnablingDeviceInput()
- 描述：三位开关是否按下

### SafetyMode getMasterboardSafetyMode()
- 描述：获取主板安全模式

### bool isFreedriveButtonPressed()
- 描述：自由驱动按钮是否按下

### bool isFreedriveIOEnabled()
- 描述：当前拖动示教IO的状态

### bool isDynamicCollisionDetectEnabled()
- 描述：当前碰撞检测模式状态

### float getToolVoltage()
- 描述：工具电压数值

### float getToolCurrent()
- 描述：工具输出电流

### ToolOutputVoltage getToolOutputVoltage()
- 描述：
- 注：
    | 模式                            | 值   | 意义                         |
    | --------------------------------- | ---- | ---------------------------- |
    | VOLTAGE_0_LEVEL  | 0 | 0V |
    | VOLTAGE_12_LEVEL | 12 | 12V |
    | VOLTAGE_24_LEVEL | 24 | 24V |

### float getToolTemperature()
- 描述：获取工具温度


### ToolMode getToolMode()
- 描述：获取工具模式
- 注：
    | 模式宏                    | 值   | 意义                   |
    | ------------------------- | ---- | ---------------------- |
    | MODE_RESET                | 235  | 重置                   |
    | MODE_SHUTTING_DOWN        | 236  | 正在关闭               |
    | MODE_POWER_OFF            | 239  | 电源关闭               |
    | MODE_NOT_RESPONDING       | 245  | 无响应                 |
    | MODE_BOOTING              | 247  | 启动中                 |
    | MODE_BOOTLOADER           | 249  | boolloader模式        |
    | MODE_FAULT                | 252  | 错误                   |
    | MODE_RUNNING              | 253  | 运行中                 |
    | MODE_IDLE                 | 255  | 待机                   |


### uint32_t getSafetyCRCNum()
- 描述：获取CRC校验参数

    enum class SafeOptMode : int8_t 
    {
        NONE = -1,
        AUTOMATIC = 0,
        MANUAL = 1
    };

### SafeOptMode getSafetyOperationalMode()
- 描述：获取安全操作模式
- 注：
    | 模式                    | 值   | 意义                   |
    | ------------------------- | ---- | ---------------------- |
    | AUTOMATIC | 0 | 自动 |
    | MANUAL | 1 | 手动 |

### std::vector<double> getCurrentElbowPos()
- 描述：获取肘部位置

### double getElbowRadius()
- 描述：获取肘部半径

### bool isToolRS485Enable()
- 描述：工具485是否打开

### uint32_t getToolRS485Baudrate()
- 描述：工具485波特率

### uint32_t getToolRS485Parity()
- 描述：工具485奇偶校验

### uint32_t getToolRS485Stopbits()
- 描述：工具485停止位

### bool isToolRS485ModbusMode()
- 描述：工具485是否为MODBUS模式

### ToolRS485Usage getToolRS485Usage()
- 描述：工具485的模式
- 注：
    | 模式                    | 值   | 意义                   |
    | ------------------------- | ---- | ---------------------- |
    | SCRIPT_MODE   | 0 | 脚本模式 |
    | DAEMON_MODE   | 1 | Daemon映射模式 |


    
    