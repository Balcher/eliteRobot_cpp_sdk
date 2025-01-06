#ifndef __ROBOT_INTERFACE_HPP__
#define __ROBOT_INTERFACE_HPP__

#include "RobotState.hpp"

#include <stdint.h>
#include <string>
#include <vector>

/// @brief �����˽ӿ��࣬�̳���RobotState
class RobotInterface : public RobotState {
   public:

    /// @brief ���ƻ������˶�
    /// @param angles ���������ת�Ƕ�(������)
    /// @param acceleration ���ٶ�
    /// @param velocity �ٶ�
    /// @param time ʱ�����(ʹ�ò�������С�Ϳ�)
    /// @param r 
    void robotMove(const std::vector<double>& angles, double acceleration,
        double velocity, double time, double r);

    /// @brief ��ȡʱ���
    /// @return ��ǰʱ���
    uint64_t getTimeStamp();
    
    /// /// @brief ���������Ƿ�ͨ��
    /// @return ���������ͨ�緵�� true,���򷵻�false
    bool isRobotPowerOn();

    /// @brief ���������Ƿ��ڽ���ֹͣ״̬
    /// @return ��������˽���ֹͣ����true�����򷵻�false
    bool isEmergencyStopped();

    /// @brief ���������Ƿ��ڱ���ֹͣ״̬
    /// @return ��������˱���ֹͣ����true�����򷵻�false
    bool isRobotProtectiveStopped();

    /// @brief �������Ƿ���������
    /// @return ��������������з���true�����򷵻�false
    bool isProgramRunning();

    /// @brief �������Ƿ�����ͣ״̬
    /// @return �����������ͣ״̬���� true�����򷵻� false
    bool isProgramPaused();

    /// @brief ������ģʽö��
    enum class RobotMode : uint8_t {
        ROBOT_MODE_DISCONNECTED = 0,          ///< �����˶Ͽ�����
        ROBOT_MODE_CONFIRM_SAFETY = 1,        ///< ����ȷ�ϰ�ȫ״̬
        ROBOT_MODE_BOOTING = 2,               ///< ������������
        ROBOT_MODE_POWER_OFF = 3,             ///< �������ѹػ�
        ROBOT_MODE_POWER_ON = 4,              ///< ��������ͨ��
        ROBOT_MODE_IDLE = 5,                  ///< �����˴��ڿ���״̬
        ROBOT_MODE_BACKDRIVE = 6,             ///< �����˴��ڷ�������ģʽ
        ROBOT_MODE_RUNNING = 7,               ///< �������������г���
        ROBOT_MODE_UPDATING_FIRMWARE = 8,     ///< ���������ڸ��¹̼�
        ROBOT_MODE_WAITING_CALIBRATION = 9    ///< ���������ڵȴ�У׼
    };
    RobotMode getRobotMode();

    /// @brief �����˿���ģʽö��
    enum class RobotControlMode : uint8_t {
        CONTROL_MODE_POSITION = 0,            ///< λ�ÿ���ģʽ
        CONTROL_MODE_TORQUE = 1               ///< ���ؿ���ģʽ

    };

    /// @brief ��ȡ��ǰ�����˿���ģʽ
    /// @return ��ǰ�����˿���ģʽ����������Ϊ RobotControlMode
    RobotControlMode getRobotControlMode();

    /// @brief ��ȡĿ���ٶȱ���
    /// @return Ŀ���ٶȵı���ֵ(0.0��1.0)
    double getTargetSpeedFraction();

    /// @brief ��ȡ�ٶ���������
    /// @return ��ǰ�ٶ���������
    double getSpeedScaling();

    /// @brief ��ȡĿ���ٶ�����
    /// @return Ŀ���ٶȵ�����ֵ
    double getTargetSpeedFractionLimit();

    /// @brief �������ٶ�ģʽö��
    enum class RobotSpeedMode : uint8_t {
        UNRESTRICTED = 0,                       ///< �������ٶ�ģʽ
        MANUAL_HIGH_SPEED = 1,                  ///< �ֶ����ٶ�ģʽ
        MANUAL_REDUCED_SPEED = 2                ///< �ֶ������ٶ�ģʽ

    };

    /// @brief ��ȡ��ǰ�������ٶ�ģʽ
    /// @return ��ǰ�������ٶ�ģʽ����������Ϊ RobotSpeedMode
    RobotSpeedMode getRobotSpeedMode();

    /// @brief ��������ϵͳ�Ƿ��ڱ���״̬
    /// @return ���������ϵͳ��������true�����򷵻�false
    bool isRobotSystemInAlarm();
    
    /// @brief ���������Ƿ��ڴ��ģʽ
    /// @return ��������˴��ڴ��ģʽ����true�����򷵻�false
    bool isInPackageMode();

    // ��ȡ�ؽ����״̬��Ϣ
    std::vector<double> getJointActualPos();              ///< ��ȡ�ؽ�ʵ��λ��
    std::vector<double> getJointTargetPos();              ///< ��ȡ�ؽ�Ŀ��λ��
    std::vector<double> getJointActualVelocity();         ///< ��ȡ�ؽ�ʵ���ٶ�
    std::vector<int32_t> getJointTargetPluse();           ///< ��ȡ�ؽ�Ŀ��������
    std::vector<int32_t> getJointActualPluse();           ///< ��ȡ�ؽ�ʵ��������
    std::vector<int32_t> getJointZeroPluse();             ///< ��ȡ�ؽ����������
    std::vector<float> getJointCurrent();                 ///< ��ȡ�ؽڵ���
    std::vector<float> getJointVoltage();                 ///< ��ȡ�ؽڵ�ѹ
    std::vector<float> getJointTemperature();             ///< ��ȡ�ؽ��¶�
    std::vector<float> getJointTorques();                 ///< ��ȡ�ؽ�Ť��

    /// @brief ö�ٶ��壺�ؽ�ģʽ��joint mode��
    enum class JointMode : uint8_t {
        MODE_RESET = 235,                                 ///< ��λģʽ
        MODE_SHUTTING_DOWN = 236,                         ///< �ػ�ģʽ
        MODE_BACKDRIVE = 238,                             ///< ������ģʽ
        MODE_POWER_OFF = 239,                             ///< �ϵ�ģʽ
        MODE_READY_FOR_POWEROFF = 240,                    ///< ׼���ϵ�ģʽ
        MODE_NOT_RESPONDING = 245,                        ///< ����Ӧģʽ
        MODE_MOTOR_INITIALISATION = 246,                  ///< �����ʼ��ģʽ
        MODE_BOOTING = 247,                               ///< ����ģʽ
        MODE_BOOTLOADER = 249,                            ///< ��������ģʽ
        MODE_VIOLATION = 251,                             ///< Υ��ģʽ
        MODE_FAULT = 252,                                 ///< ����ģʽ
        MODE_RUNNING = 253,                               ///< ����ģʽ
        MODE_IDLE = 255                                   ///< ����ģʽ

    };
    std::vector<JointMode> getJointMode();                ///< ��ȡ�ؽڹ���ģʽ

    std::vector<double> getTcpPosition();                 ///< ��ȡTCPλ��
    std::vector<double> getTcpOffset();                   ///< ��ȡTCPƫ����

    std::vector<double> getLimitMinJoint();               ///< ��ȡ�ؽ���С����
    std::vector<double> getLimitMaxJoint();               ///< ��ȡ�ؽ��������
    std::vector<double> getMaxVelocityJoint();            ///< ��ȡ�ؽ�����ٶ�
    std::vector<double> getMaxAccJoint();                 ///< ��ȡ�ؽ������ٶ�
    double getDefaultVelocityJoint();                     ///< ��ȡĬ�Ϲؽ��ٶ�
    double getDefaultAccJoint();                          ///< ��ȡĬ�Ϲؽڼ��ٶ�
    double getDefaultToolVelocity();                      ///< ��ȡĬ�Ϲ����ٶ�
    double getDefaultToolAcc();                           ///< ��ȡĬ�Ϲ��߼��ٶ�
    double getEqRadius();                                 ///< ��ȡ��Ч�뾶
    std::vector<double> getDhAJoint();                    ///< ��ȡDH���� a_i
    std::vector<double> getDhDJoint();                    ///< ��ȡDH���� d_i
    std::vector<double> getDhAlphaJoint();                ///< ��ȡDH���� \alpha_i
    
    // ��ȡӲ���汾��������Ϣ
    uint32_t getBoardVersion();                           ///< ��ȡ�忨�汾
    uint32_t getControlBoxType();                         ///< ��ȡ����������

    /// @brief ö�����ͣ�����������
    enum class RobotType : uint32_t {
        ROBOT_TYPE_6203 = 6203,                          ///< �������ͺ� 6203
        ROBOT_TYPE_6206 = 6206,                          ///< �������ͺ� 6206
        ROBOT_TYPE_6212 = 6212                           ///< �������ͺ� 6212

    };
    RobotType getRobotType();

    /// @brief ö�ٶ��壺�����˽ṹ
    enum class RobotStruct : uint32_t {
        ROBOT_STRUCTURE_TYPE_60 = 60,                    ///< �ṹ���� 60
        ROBOT_STRUCTURE_TYPE_62 = 62,                    ///< �ṹ���� 62
        ROBOT_STRUCTURE_TYPE_70 = 70                     ///< �ṹ���� 70

    };
    RobotStruct getRobotStruct();

    std::vector<bool> getStandardDigitalInput();                        ///< ��ȡ��׼��������
    std::vector<bool> getStandardDigitalOutput();                       ///< ��ȡ��׼�������
    std::vector<bool> getConfigureDigitalInput();                       ///< ��ȡ������������
    std::vector<bool> getConfigureDigitalOutput();                      ///< ��ȡ�����������
    std::vector<bool> getToolDigitalInput();                            ///< ��ȡ������������
    std::vector<bool> getToolDigitalOutput();                           ///< ��ȡ�����������
    
    enum class AnalogMode : uint8_t { CURRENT = 0, VOLTAGE = 1 };       ///< 
    
    std::vector<AnalogMode> getStandardAnalogOutputMode();              ///< ��ȡ��׼ģ�����ģʽ
    std::vector<AnalogMode> getStandardAnalogInputMode();               ///< ��ȡ��׼ģ������ģʽ
    AnalogMode getToolAnalogOutputMode();                               ///< ��ȡ����ģ�����ģʽ
    AnalogMode getToolAnalogInputMode();                                ///< ��ȡ����ģ������ģʽ
    std::vector<double> getStandardAnalogOutputValue();                 ///< ��ȡ��׼ģ�����ֵ
    std::vector<double> getStandardAnalogInputValue();                  ///< ��ȡ��׼ģ������ֵ
    double getToolAnalogOutputValue();                                  ///< ��ȡ����ģ�����ֵ
    double getToolAnalogInputValue();                                   ///< ��ȡ����ģ������ֵ
    
    // ��ȡ�����˵�������
    float getBoardTemperature();                                        ///< ��ȡ�忨�¶�
    float getRobotVoltage();                                            ///< ��ȡ�����˵�ѹ
    float getRobotCurrent();                                            ///< ��ȡ�����˵���
    float getIOCurrent();                                               ///< ��ȡIO����

    // ö�ٶ��壺��ȫģʽ
    enum class SafetyMode : uint8_t {
        SAFETY_MODE_NORMAL = 1,                                         ///< ����ģʽ
        SAFETY_MODE_REDUCED = 2,                                        ///< ����ģʽ
        SAFETY_MODE_PROTECTIVE_STOP = 3,                                ///< ����ֹͣģʽ
        SAFETY_MODE_RECOVERY = 4,                                       ///< �ָ�ģʽ
        SAFETY_MODE_SAFEGUARD_STOP = 5,                                 ///< ��ȫֹͣģʽ
        SAFETY_MODE_SYSTEM_EMERGENCY_STOP = 6,                          ///< ϵͳ����ֹͣģʽ
        SAFETY_MODE_ROBOT_EMERGENCY_STOP = 7,                           ///< �����˽���ֹͣģʽ
        SAFETY_MODE_VIOLATION = 8,                                      ///< Υ��ģʽ
        SAFETY_MODE_FAULT = 9,                                          ///< ����ģʽ
        SAFETY_MODE_VALIDATE_JOINT_ID = 10,                             ///< ��֤�ؽ�ID
        SAFETY_MODE_UNDEFINED_SAFETY_MODE = 11,                         ///< δ����İ�ȫģʽ
        SAFETY_MODE_AUTOMATIC_MODE_SAFEGUARD_STOP = 12,                 ///< �Զ�ģʽ��ȫֹͣ
        SAFETY_MODE_SYSTEM_THREE_POSITION_ENABLING_STOP = 13            ///< ϵͳ��λ������ֹͣ
    };
    
    // ��ȡ��ȫģʽ
    SafetyMode getBordSafeMode();                                       ///< ��ȡ���尲ȫģʽ
    bool isRobotInReducedMode();                                        ///< �жϻ������Ƿ��ڽ���ģʽ
    bool getOperationalModeSelectorInput();                             ///< ��ȡ����ģʽѡ��������
    bool getThreepositionEnablingDeviceInput();                         ///< ��ȡ��λ�������豸����
    SafetyMode getMasterboardSafetyMode();                              ///< ��ȡ���尲ȫģʽ

    // ��ȡ�����������״̬
    bool isFreedriveButtonPressed();                                    ///< �ж�����������ť�Ƿ���
    bool isFreedriveIOEnabled();                                        ///< �ж���������IO�Ƿ�����
    bool isDynamicCollisionDetectEnabled();                             ///< �ж϶�̬��ײ����Ƿ�����

    // ��ȡ������صĵ�������
    float getToolVoltage();                                             ///< ��ȡ���ߵĵ�ѹֵ
    float getToolCurrent();                                             ///< ��ȡ���ߵĵ���ֵ

    // ö�ٶ��壺���������ѹ�ȼ�
    enum class ToolOutputVoltage : uint8_t { 
        VOLTAGE_0_LEVEL = 0,                                            ///< ���������ѹΪ 0v
        VOLTAGE_12_LEVEL = 12,                                          ///< ���������ѹΪ 12v
        VOLTAGE_24_LEVEL = 24                                           ///< ���������ѹΪ 24v
    };

    // ��ȡ���ߵ���״̬
    ToolOutputVoltage getToolOutputVoltage();                           ///< ��ȡ���ߵ������ѹ�ȼ�
    float getToolTemperature();                                         ///< ��ȡ���ߵ��¶�

    // ����ؽ�ģʽ��JointMode����Ϊ����ģʽ(ToolMode)�ı���
    typedef JointMode ToolMode;                                         
    ToolMode getToolMode();                                             ///< ���ع��ߵĵ�ǰģʽ

    // ��ȡ��ȫ��ص���Ϣ
    uint32_t getSafetyCRCNum();                                         ///< ��ȡ��ȫ��ص�CRCУ����
    
    // ö�ٶ��壺��ȫ����ģʽ
    enum class SafeOptMode : int8_t { 
        NONE = -1,                                                      ///< �ް�ȫ����ģʽ
        AUTOMATIC = 0,                                                  ///< �Զ���ȫģʽ
        MANUAL = 1                                                      ///< �ֶ���ȫģʽ
    };
    
    // ��ȡ��ǰ�İ�ȫ����ģʽ
    SafeOptMode getSafetyOperationalMode();                             ///< ���ػ����˵�ǰ�İ�ȫ����ģʽ

    // ��ȡ�ⲿ��ز���
    std::vector<double> getCurrentElbowPos();                           ///< ��ȡ��ǰ�ⲿλ�õ�����(������ 3D ����)
    double getElbowRadius();                                            ///< ��ȡ�ⲿ�İ뾶��Ϣ

    // ���� RS485 �ӿ���ع���
    bool isToolRS485Enable();                                           ///< �жϹ��ߵ� RS485 �ӿ��Ƿ�����
    uint32_t getToolRS485Baudrate();                                    ///< ��ȡ���� RS485 �Ĳ�����
    uint32_t getToolRS485Parity();                                      ///< ��ȡ���� RS485 ��У��λ����
    uint32_t getToolRS485Stopbits();                                    ///< ��ȡ���� RS485 ��ֹͣλ����
    bool isToolRS485ModbusMode();                                       ///< �жϹ����Ƿ��� Modbus ģʽ

    // ö�ٶ��壺���� RS485 ��ʹ��ģʽ
    enum class ToolRS485Usage : uint8_t { 
        SCRIPT_MODE,                                                    ///< RS485 �ű�
        DAEMON_MODE                                                     ///< RS485 �ػ�����ģʽ
    };
    ToolRS485Usage getToolRS485Usage();
    
    // ģ�庯������״̬�б��л�ȡָ�����͵�״ֵ̬
    template <typename T>
    std::vector<T> getStatesVector(const char* state_list[], int list_size) {
        std::vector<T> result;
        // ����״̬�б����� getItemData ������ȡÿ��״ֵ̬���洢�����������
        for (int i = 0; i < list_size; i++) {
            result.push_back(getItemData<T>(state_list[i], nullptr));
        }
        return result;
    }

    RobotInterface();
    ~RobotInterface();
};

#endif