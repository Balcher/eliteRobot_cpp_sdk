#include "RobotInterface.hpp"
#include <sstream>

void RobotInterface::robotJointMove(const std::vector<double> &angles, double acceleration, double velocity, double time, double r)
{
    std::ostringstream script;

    script << "def move():\n";
    script << "\tmovej([";

    // 添加角度值
    for (size_t i = 0; i < angles.size(); ++i)
    {
        script << angles[i];
        if (i < angles.size() - 1)
        {
            script << ",";
        }
    }

    script << "],a=" << acceleration << ",v=" << velocity << ",t=" << time << ",r=" << r << ")\n";
    script << "end";

    this->sendScript(script.str());
}

void RobotInterface::robotLinearMove(const std::vector<double> &poses, double acceleration,
                                double velocity, double time, double r)
{
    std::ostringstream script;

    script << "def move():\n";
    script << "\tmovel([";

    // 添加目标点的工具位姿
    for (size_t i = 0; i < poses.size(); ++i)
    {
        script << poses[i];
        if (i < poses.size() - 1)
        {
            script << ",";
        }
    }

    script << "],a=" << acceleration << ",v=" << velocity << ",t=" << time << ",r=" << r << ")\n";
    script << "end";

    this->sendScript(script.str());
}

RobotInterface::RobotInterface() {}

RobotInterface::~RobotInterface() {}

uint64_t RobotInterface::getTimeStamp()
{
    uint64_t timestamp = 0;
    getItemData("timestamp", &timestamp);
    return timestamp;
}

bool RobotInterface::isRobotPowerOn() { return getItemData<bool>("Robot_mode_is_robot_power_on", nullptr); }

bool RobotInterface::isEmergencyStopped() { return getItemData<bool>("Robot_mode_is_emergency_stopped", nullptr); }

bool RobotInterface::isRobotProtectiveStopped() { return getItemData<bool>("Robot_mode_is_robot_protective_stopped", nullptr); }

bool RobotInterface::isProgramRunning() { return getItemData<bool>("Robot_mode_is_program_running", nullptr); }

bool RobotInterface::isProgramPaused() { return getItemData<bool>("Robot_mode_is_program_paused", nullptr); }

RobotInterface::RobotMode RobotInterface::getRobotMode()
{
    return (RobotMode)getItemData<uint8_t>("Robot_mode_get_robot_mode", nullptr);
}

RobotInterface::RobotControlMode RobotInterface::getRobotControlMode()
{
    return (RobotControlMode)getItemData<uint8_t>("Robot_mode_get_robot_control_mode", nullptr);
}

double RobotInterface::getTargetSpeedFraction() { return getItemData<double>("Robot_mode_get_target_speed_fraction", nullptr); }

double RobotInterface::getSpeedScaling() { return getItemData<double>("Robot_mode_get_speed_scaling", nullptr); }

double RobotInterface::getTargetSpeedFractionLimit()
{
    return getItemData<double>("Robot_mode_get_target_speed_fraction_limit", nullptr);
}

RobotInterface::RobotSpeedMode RobotInterface::getRobotSpeedMode()
{
    return (RobotSpeedMode)getItemData<uint8_t>("Robot_mode_get_robot_speed_mode", nullptr);
}

bool RobotInterface::isRobotSystemInAlarm() { return getItemData<bool>("Robot_mode_is_robot_system_in_alarm", nullptr); }

bool RobotInterface::isInPackageMode() { return getItemData<bool>("Robot_mode_is_in_package_mode", nullptr); }

std::vector<double> RobotInterface::getJointActualPos()
{
    const char *joints[] = {"Joint_actual_joint0", "Joint_actual_joint1", "Joint_actual_joint2",
                            "Joint_actual_joint3", "Joint_actual_joint4", "Joint_actual_joint5"};
    return getStatesVector<double>(joints, 6);
}

std::vector<double> RobotInterface::getJointTargetPos()
{
    const char *joints[] = {"Joint_target_joint0", "Joint_target_joint1", "Joint_target_joint2",
                            "Joint_target_joint3", "Joint_target_joint4", "Joint_target_joint5"};
    return getStatesVector<double>(joints, 6);
}

std::vector<double> RobotInterface::getJointActualVelocity()
{
    const char *joints[] = {"Joint_target_pluse0", "Joint_target_pluse1", "Joint_target_pluse2",
                            "Joint_target_pluse3", "Joint_target_pluse4", "Joint_target_pluse5"};
    return getStatesVector<double>(joints, 6);
}

std::vector<int32_t> RobotInterface::getJointTargetPluse()
{
    const char *joints[] = {"Joint_target_pluse0", "Joint_target_pluse1", "Joint_target_pluse2",
                            "Joint_target_pluse3", "Joint_target_pluse4", "Joint_target_pluse5"};
    return getStatesVector<int32_t>(joints, 6);
}

std::vector<int32_t> RobotInterface::getJointActualPluse()
{
    const char *joints[] = {"Joint_actual_pluse0", "Joint_actual_pluse1", "Joint_actual_pluse2",
                            "Joint_actual_pluse3", "Joint_actual_pluse4", "Joint_actual_pluse5"};
    return getStatesVector<int32_t>(joints, 6);
}

std::vector<int32_t> RobotInterface::getJointZeroPluse()
{
    const char *joints[] = {"Joint_zero_pluse0", "Joint_zero_pluse1", "Joint_zero_pluse2",
                            "Joint_zero_pluse3", "Joint_zero_pluse4", "Joint_zero_pluse5"};
    return getStatesVector<int32_t>(joints, 6);
}

std::vector<float> RobotInterface::getJointCurrent()
{
    const char *joints[] = {"Joint_current0", "Joint_current1", "Joint_current2",
                            "Joint_current3", "Joint_current4", "Joint_current5"};
    return getStatesVector<float>(joints, 6);
}

std::vector<float> RobotInterface::getJointVoltage()
{
    const char *joints[] = {"Joint_voltage0", "Joint_voltage1", "Joint_voltage2",
                            "Joint_voltage3", "Joint_voltage4", "Joint_voltage5"};
    return getStatesVector<float>(joints, 6);
}

std::vector<float> RobotInterface::getJointTemperature()
{
    const char *joints[] = {"Joint_temperature0", "Joint_temperature1", "Joint_temperature2",
                            "Joint_temperature3", "Joint_temperature4", "Joint_temperature5"};
    return getStatesVector<float>(joints, 6);
}

std::vector<float> RobotInterface::getJointTorques()
{
    const char *joints[] = {"Joint_torques0", "Joint_torques1", "Joint_torques2",
                            "Joint_torques3", "Joint_torques4", "Joint_torques5"};
    return getStatesVector<float>(joints, 6);
}

std::vector<RobotInterface::JointMode> RobotInterface::getJointMode()
{
    const char *joints[] = {"Joint_torques0", "Joint_torques1", "Joint_torques2",
                            "Joint_torques3", "Joint_torques4", "Joint_torques5"};
    return getStatesVector<JointMode>(joints, 6);
}

std::vector<double> RobotInterface::getTcpPosition()
{
    std::vector<double> result;
    double value = getItemData<double>("Cartesian_tcp_x", nullptr);
    result.push_back(value);
    value = getItemData<double>("Cartesian_tcp_y", nullptr);
    result.push_back(value);
    value = getItemData<double>("Cartesian_tcp_z", nullptr);
    result.push_back(value);
    value = getItemData<double>("Cartesian_rot_x", nullptr);
    result.push_back(value);
    value = getItemData<double>("Cartesian_rot_y", nullptr);
    result.push_back(value);
    value = getItemData<double>("Cartesian_rot_z", nullptr);
    result.push_back(value);
    return result;
}

std::vector<double> RobotInterface::getTcpOffset()
{
    std::vector<double> result;
    double value = getItemData<double>("Cartesian_offset_px", nullptr);
    result.push_back(value);
    value = getItemData<double>("Cartesian_offset_py", nullptr);
    result.push_back(value);
    value = getItemData<double>("Cartesian_offset_pz", nullptr);
    result.push_back(value);
    value = getItemData<double>("Cartesian_offset_rotx", nullptr);
    result.push_back(value);
    value = getItemData<double>("Cartesian_offset_roty", nullptr);
    result.push_back(value);
    value = getItemData<double>("Cartesian_offset_rotz", nullptr);
    result.push_back(value);
    return result;
}

std::vector<double> RobotInterface::getLimitMinJoint()
{
    const char *joints[] = {"Robot_configure_limit_min_joint_x0", "Robot_configure_limit_min_joint_x1",
                            "Robot_configure_limit_min_joint_x2", "Robot_configure_limit_min_joint_x3",
                            "Robot_configure_limit_min_joint_x4", "Robot_configure_limit_min_joint_x5"};
    return getStatesVector<double>(joints, 6);
}

std::vector<double> RobotInterface::getLimitMaxJoint()
{
    const char *joints[] = {"Robot_configure_limit_max_joint_x0", "Robot_configure_limit_max_joint_x1",
                            "Robot_configure_limit_max_joint_x2", "Robot_configure_limit_max_joint_x3",
                            "Robot_configure_limit_max_joint_x4", "Robot_configure_limit_max_joint_x5"};
    return getStatesVector<double>(joints, 6);
}

std::vector<double> RobotInterface::getMaxVelocityJoint()
{
    const char *joints[] = {"Robot_configure_max_velocity_joint_x0", "Robot_configure_max_velocity_joint_x1",
                            "Robot_configure_max_velocity_joint_x2", "Robot_configure_max_velocity_joint_x3",
                            "Robot_configure_max_velocity_joint_x4", "Robot_configure_max_velocity_joint_x5"};
    return getStatesVector<double>(joints, 6);
}

std::vector<double> RobotInterface::getMaxAccJoint()
{
    const char *joints[] = {"Robot_configure_max_acc_joint_x0", "Robot_configure_max_acc_joint_x1",
                            "Robot_configure_max_acc_joint_x2", "Robot_configure_max_acc_joint_x3",
                            "Robot_configure_max_acc_joint_x4", "Robot_configure_max_acc_joint_x5"};
    return getStatesVector<double>(joints, 6);
}

double RobotInterface::getDefaultVelocityJoint() { return getItemData<double>("Robot_configure_default_velocity_joint", nullptr); }

double RobotInterface::getDefaultAccJoint() { return getItemData<double>("Robot_configure_default_acc_joint", nullptr); }

double RobotInterface::getDefaultToolVelocity() { return getItemData<double>("Robot_configure_default_tool_velocity", nullptr); }

double RobotInterface::getDefaultToolAcc() { return getItemData<double>("Robot_configure_default_tool_acc", nullptr); }

double RobotInterface::getEqRadius() { return getItemData<double>("Robot_configure_eq_radius", nullptr); }

std::vector<double> RobotInterface::getDhAJoint()
{
    const char *joints[] = {"Robot_configure_dh_a_joint_x0", "Robot_configure_dh_a_joint_x1", "Robot_configure_dh_a_joint_x2",
                            "Robot_configure_dh_a_joint_x3", "Robot_configure_dh_a_joint_x4", "Robot_configure_dh_a_joint_x5"};
    return getStatesVector<double>(joints, 6);
}

std::vector<double> RobotInterface::getDhDJoint()
{
    const char *joints[] = {"Robot_configure_dh_d_joint_d0", "Robot_configure_dh_d_joint_d1", "Robot_configure_dh_d_joint_d2",
                            "Robot_configure_dh_d_joint_d3", "Robot_configure_dh_d_joint_d4", "Robot_configure_dh_d_joint_d5"};
    return getStatesVector<double>(joints, 6);
}

std::vector<double> RobotInterface::getDhAlphaJoint()
{
    const char *joints[] = {"Robot_configure_dh_alpha_joint_x0", "Robot_configure_dh_alpha_joint_x1",
                            "Robot_configure_dh_alpha_joint_x2", "Robot_configure_dh_alpha_joint_x3",
                            "Robot_configure_dh_alpha_joint_x4", "Robot_configure_dh_alpha_joint_x5"};
    return getStatesVector<double>(joints, 6);
}

uint32_t RobotInterface::getBoardVersion() { return getItemData<uint32_t>("Robot_configure_board_version", nullptr); }

uint32_t RobotInterface::getControlBoxType() { return getItemData<uint32_t>("Robot_configure_control_box_type", nullptr); }

RobotInterface::RobotType RobotInterface::getRobotType() { return getItemData<RobotType>("Robot_configure_robot_type", nullptr); }

RobotInterface::RobotStruct RobotInterface::getRobotStruct()
{
    return getItemData<RobotStruct>("Robot_configure_robot_struct", nullptr);
}

#pragma pack(1)
struct DigitalBits
{
    bool standard0 : 1;
    bool standard1 : 1;
    bool standard2 : 1;
    bool standard3 : 1;
    bool standard4 : 1;
    bool standard5 : 1;
    bool standard6 : 1;
    bool standard7 : 1;
    bool standard8 : 1;
    bool standard9 : 1;
    bool standard10 : 1;
    bool standard11 : 1;
    bool standard12 : 1;
    bool standard13 : 1;
    bool standard14 : 1;
    bool standard15 : 1;

    bool configure0 : 1;
    bool configure1 : 1;
    bool configure2 : 1;
    bool configure3 : 1;
    bool configure4 : 1;
    bool configure5 : 1;
    bool configure6 : 1;
    bool configure7 : 1;

    bool tool0 : 1;
    bool tool1 : 1;
    bool tool2 : 1;
    bool tool3 : 1;

    bool reserver0 : 1;
    bool reserver1 : 1;
    bool reserver2 : 1;
    bool reserver3 : 1;
};
#pragma pack()

std::vector<bool> RobotInterface::getStandardDigitalInput()
{
    uint32_t digital = getItemData<uint32_t>("Robot_motherboard_digital_input_bits", nullptr);
    DigitalBits *bits = (DigitalBits *)&digital;
    std::vector<bool> result(16);
    result.push_back(bits->standard0);
    result.push_back(bits->standard2);
    result.push_back(bits->standard3);
    result.push_back(bits->standard4);
    result.push_back(bits->standard5);
    result.push_back(bits->standard6);
    result.push_back(bits->standard7);
    result.push_back(bits->standard8);
    result.push_back(bits->standard9);
    result.push_back(bits->standard10);
    result.push_back(bits->standard11);
    result.push_back(bits->standard12);
    result.push_back(bits->standard13);
    result.push_back(bits->standard14);
    result.push_back(bits->standard15);
    return result;
}

std::vector<bool> RobotInterface::getStandardDigitalOutput()
{
    uint32_t digital = getItemData<uint32_t>("Robot_motherboard_digital_output_bits", nullptr);
    DigitalBits *bits = (DigitalBits *)&digital;
    std::vector<bool> result(16);
    result.push_back(bits->standard0);
    result.push_back(bits->standard2);
    result.push_back(bits->standard3);
    result.push_back(bits->standard4);
    result.push_back(bits->standard5);
    result.push_back(bits->standard6);
    result.push_back(bits->standard7);
    result.push_back(bits->standard8);
    result.push_back(bits->standard9);
    result.push_back(bits->standard10);
    result.push_back(bits->standard11);
    result.push_back(bits->standard12);
    result.push_back(bits->standard13);
    result.push_back(bits->standard14);
    result.push_back(bits->standard15);
    return result;
}

std::vector<bool> RobotInterface::getConfigureDigitalInput()
{
    uint32_t digital = getItemData<uint32_t>("Robot_motherboard_digital_input_bits", nullptr);
    DigitalBits *bits = (DigitalBits *)&digital;
    std::vector<bool> result(8);
    result.push_back(bits->configure0);
    result.push_back(bits->configure1);
    result.push_back(bits->configure2);
    result.push_back(bits->configure3);
    result.push_back(bits->configure4);
    result.push_back(bits->configure5);
    result.push_back(bits->configure6);
    result.push_back(bits->configure7);
    return result;
}

std::vector<bool> RobotInterface::getConfigureDigitalOutput()
{
    uint32_t digital = getItemData<uint32_t>("Robot_motherboard_digital_output_bits", nullptr);
    DigitalBits *bits = (DigitalBits *)&digital;
    std::vector<bool> result(8);
    result.push_back(bits->configure0);
    result.push_back(bits->configure1);
    result.push_back(bits->configure2);
    result.push_back(bits->configure3);
    result.push_back(bits->configure4);
    result.push_back(bits->configure5);
    result.push_back(bits->configure6);
    result.push_back(bits->configure7);
    return result;
}

std::vector<bool> RobotInterface::getToolDigitalInput()
{
    uint32_t digital = getItemData<uint32_t>("Robot_motherboard_digital_input_bits", nullptr);
    DigitalBits *bits = (DigitalBits *)&digital;
    std::vector<bool> result(4);
    result.push_back(bits->tool0);
    result.push_back(bits->tool1);
    result.push_back(bits->tool2);
    result.push_back(bits->tool3);
    return result;
}

std::vector<bool> RobotInterface::getToolDigitalOutput()
{
    uint32_t digital = getItemData<uint32_t>("Robot_motherboard_digital_output_bits", nullptr);
    DigitalBits *bits = (DigitalBits *)&digital;
    std::vector<bool> result(4);
    result.push_back(bits->tool0);
    result.push_back(bits->tool1);
    result.push_back(bits->tool2);
    result.push_back(bits->tool3);
    return result;
}

std::vector<RobotInterface::AnalogMode> RobotInterface::getStandardAnalogOutputMode()
{
    std::vector<AnalogMode> result;
    result.push_back(getItemData<AnalogMode>("Robot_motherboard_standard_analog_output_domain0", nullptr));
    result.push_back(getItemData<AnalogMode>("Robot_motherboard_standard_analog_output_domain1", nullptr));
    return result;
}

std::vector<RobotInterface::AnalogMode> RobotInterface::getStandardAnalogInputMode()
{
    std::vector<AnalogMode> result;
    result.push_back(getItemData<AnalogMode>("Robot_motherboard_standard_analog_output_domain0", nullptr));
    result.push_back(getItemData<AnalogMode>("Robot_motherboard_standard_analog_output_domain1", nullptr));
    return result;
}

RobotInterface::AnalogMode RobotInterface::getToolAnalogOutputMode()
{
    return getItemData<AnalogMode>("Robot_motherboard_tool_analog_output_domain", nullptr);
}

RobotInterface::AnalogMode RobotInterface::getToolAnalogInputMode()
{
    return getItemData<AnalogMode>("Robot_motherboard_tool_analog_input_domain", nullptr);
}

std::vector<double> RobotInterface::getStandardAnalogOutputValue()
{
    std::vector<double> result;
    result.push_back(getItemData<double>("Robot_motherboard_standard_analog_output_value0", nullptr));
    result.push_back(getItemData<double>("Robot_motherboard_standard_analog_output_value1", nullptr));
    return result;
}

std::vector<double> RobotInterface::getStandardAnalogInputValue()
{
    std::vector<double> result;
    result.push_back(getItemData<double>("Robot_motherboard_standard_analog_input_value0", nullptr));
    result.push_back(getItemData<double>("Robot_motherboard_standard_analog_input_value1", nullptr));
    return result;
}

double RobotInterface::getToolAnalogOutputValue()
{
    return getItemData<double>("Robot_motherboard_tool_analog_output_value", nullptr);
}

double RobotInterface::getToolAnalogInputValue()
{
    return getItemData<double>("Robot_motherboard_tool_analog_input_value", nullptr);
}

float RobotInterface::getBoardTemperature() { return getItemData<float>("Robot_motherboard_bord_temperature", nullptr); }

float RobotInterface::getRobotVoltage() { return getItemData<float>("Robot_motherboard_robot_voltage", nullptr); }

float RobotInterface::getRobotCurrent() { return getItemData<float>("Robot_motherboard_robot_current", nullptr); }

float RobotInterface::getIOCurrent() { return getItemData<float>("Robot_motherboard_io_current", nullptr); }

RobotInterface::SafetyMode RobotInterface::getBordSafeMode()
{
    return getItemData<SafetyMode>("Robot_motherboard_bord_safe_mode", nullptr);
}

bool RobotInterface::isRobotInReducedMode() { return getItemData<bool>("Robot_motherboard_is_robot_in_reduced_mode", nullptr); }

bool RobotInterface::getOperationalModeSelectorInput()
{
    return getItemData<bool>("Robot_motherboard_get_operational_mode_selector_input", nullptr);
}

bool RobotInterface::getThreepositionEnablingDeviceInput()
{
    return getItemData<bool>("Robot_motherboard_get_threeposition_enabling_device_input", nullptr);
}

RobotInterface::SafetyMode RobotInterface::getMasterboardSafetyMode()
{
    return getItemData<SafetyMode>("Robot_motherboard_masterboard_safety_mode", nullptr);
}

bool RobotInterface::isFreedriveButtonPressed()
{
    return getItemData<bool>("Robot_motherboard_is_freedrive_button_pressed", nullptr);
}

bool RobotInterface::isFreedriveIOEnabled() { return getItemData<bool>("Robot_motherboard_is_freedrive_io_enabled", nullptr); }

bool RobotInterface::isDynamicCollisionDetectEnabled()
{
    return getItemData<bool>("Robot_motherboard_is_dynamic_collision_detect_enabled", nullptr);
}

float RobotInterface::getToolVoltage() { return getItemData<float>("Robot_tool_data_tool_voltage", nullptr); }

float RobotInterface::getToolCurrent() { return getItemData<float>("Robot_tool_data_tool_current", nullptr); }

RobotInterface::ToolOutputVoltage RobotInterface::getToolOutputVoltage()
{
    return getItemData<ToolOutputVoltage>("Robot_tool_data_tool_output_voltage", nullptr);
}

float RobotInterface::getToolTemperature() { return getItemData<float>("Robot_tool_data_tool_temperature", nullptr); }

RobotInterface::ToolMode RobotInterface::getToolMode() { return getItemData<ToolMode>("Robot_tool_data_tool_mode", nullptr); }

uint32_t RobotInterface::getSafetyCRCNum() { return getItemData<uint32_t>("Robot_satety_mode_safety_crc_num", nullptr); }

RobotInterface::SafeOptMode RobotInterface::getSafetyOperationalMode()
{
    return getItemData<SafeOptMode>("Robot_satety_mode_safety_operational_mode", nullptr);
}

std::vector<double> RobotInterface::getCurrentElbowPos()
{
    std::vector<double> result(2);
    result.push_back(getItemData<double>("Robot_satety_mode_current_elbow_position_x", nullptr));
    result.push_back(getItemData<double>("Robot_satety_mode_current_elbow_position_y", nullptr));
    result.push_back(getItemData<double>("Robot_satety_mode_current_elbow_position_z", nullptr));
    return result;
}

double RobotInterface::getElbowRadius() { return getItemData<double>("Robot_satety_mode_elbow_radius", nullptr); }

bool RobotInterface::isToolRS485Enable() { return getItemData<bool>("Robot_communication_is_enable", nullptr); }

uint32_t RobotInterface::getToolRS485Baudrate() { return getItemData<bool>("Robot_communication_baudrate", nullptr); }

uint32_t RobotInterface::getToolRS485Parity() { return getItemData<bool>("Robot_communication_parity", nullptr); }

uint32_t RobotInterface::getToolRS485Stopbits() { return getItemData<bool>("Robot_communication_stopbits", nullptr); }

bool RobotInterface::isToolRS485ModbusMode() { return getItemData<bool>("Robot_communication_tci_modbus_status", nullptr); }

RobotInterface::ToolRS485Usage RobotInterface::getToolRS485Usage()
{
    return getItemData<ToolRS485Usage>("Robot_communication_tci_usage", nullptr);
}
