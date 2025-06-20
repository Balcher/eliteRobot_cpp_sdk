#include "rtsiReceive.h"

RtsiReceive::RtsiReceive(const std::string &ip, int jointRate, int payloadRate)
{
    m_jointRate = jointRate;
    rt = std::make_unique<Rtsi>();
    if (!rt->connect(ip))
    {
        std::cout << "Connect fail" << std::endl;
        return;
    }
    if (!rt->versionCheck())
    {
        std::cout << "Version check fail" << std::endl;
        return;
    }
    out_recipe1 = rt->outputSubscribe("payload_mass,payload_cog", payloadRate);
    out_recipe2 = rt->outputSubscribe("timestamp,actual_joint_positions", jointRate);
    rt->sendTextMessage("New RTSI Connect", "RTSI CPP Client", Rtsi::MessageType::INFO_MESSAGE);

    setTCPOffset(0.0, -0.045, 0.065, 0.0, 0.0, 0.0);
}

RtsiReceive::~RtsiReceive()
{
    RtsiStop();
    if (m_update_thread.joinable())
    {
        m_update_thread.join();
    }
    std::cout << "RtsiReceive Destructor" << std::endl;
}

void RtsiReceive::RtsiStart()
{
    rt->start();
    m_running = true;
    m_update_thread = std::thread(&RtsiReceive::updateJointPosition, this);
}

void RtsiReceive::RtsiStop()
{
    m_running = false;
    rt->pause();
    rt->disconnect();
}

std::vector<double> RtsiReceive::GetActualJointPositions()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_last_joint_positions;
}

std::vector<double> RtsiReceive::GetPayloadCog()
{
    Rtsi::DataRecipePtr &recipe = rt->getOutputDataToRecipe();
    if (recipe->getID() == out_recipe1->getID())
    {
        std::vector<double> payload_cog;
        for (size_t i = 0; i < 3; i++)
        {
            payload_cog.push_back((*recipe)["payload_cog"].value.v3d[i]);
        }
        return payload_cog;
    }
    return {};
}

void RtsiReceive::setTCPOffset(double x, double y, double z, double roll, double pitch, double yaw)
{
    tcp_offset_x = x;
    tcp_offset_y = y;
    tcp_offset_z = z;
    tcp_offset_roll = roll;
    tcp_offset_pitch = pitch;
    tcp_offset_yaw = yaw;
}

std::vector<double> RtsiReceive::getTcpPosition(const std::vector<double> &joint_positions)
{
    // 欧拉角顺序是ZYX
    Eigen::Matrix4d tcp_matrix = forwardKinematics(joint_positions);
    return {tcp_matrix(0, 3), tcp_matrix(1, 3), tcp_matrix(2, 3),
            std::atan2(tcp_matrix(2, 1), tcp_matrix(2, 2)),                                                                      // roll
            std::atan2(-tcp_matrix(2, 0), std::sqrt(tcp_matrix(2, 1) * tcp_matrix(2, 1) + tcp_matrix(2, 2) * tcp_matrix(2, 2))), // pitch
            std::atan2(tcp_matrix(1, 0), tcp_matrix(0, 0))};                                                                     // yaw
}

void RtsiReceive::setEventDrivenJointUpdataCallback(JointUpdataCallback callback)
{
    event_driven_joint_update_callback = callback;
}

void RtsiReceive::setRealTimeJointUpdataCallback(JointUpdataCallback callback)
{
    real_time_joint_update_callback = callback;
}

std::vector<std::vector<double>> RtsiReceive::getJointActualPosition(std::vector<double> degrees)
{

    Eigen::Matrix4d T1 = computeTransformationMatrix(dh_parameters[0], degrees[0]);
    Eigen::Matrix4d T2 = computeTransformationMatrix(dh_parameters[1], degrees[1]);
    Eigen::Matrix4d T3 = computeTransformationMatrix(dh_parameters[2], degrees[2]);
    Eigen::Matrix4d T4 = computeTransformationMatrix(dh_parameters[3], degrees[3]);
    Eigen::Matrix4d T5 = computeTransformationMatrix(dh_parameters[4], degrees[4]);
    Eigen::Matrix4d T6 = computeTransformationMatrix(dh_parameters[5], degrees[5]);

    std::vector<std::vector<double>> jointActualPos;

    jointActualPos.push_back(matrixToVector(T1));
    jointActualPos.push_back(matrixToVector(T2));
    jointActualPos.push_back(matrixToVector(T3));
    jointActualPos.push_back(matrixToVector(T4));
    jointActualPos.push_back(matrixToVector(T5));
    jointActualPos.push_back(matrixToVector(T6));

    return jointActualPos;
}

Eigen::Matrix4d RtsiReceive::forwardKinematics(const std::vector<double> &joint_positions)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < 5; i++)
    {
        T = T * computeTransformationMatrix(dh_parameters[i], joint_positions[i]);
    }

    // 处理末端TCP情况，相当于基于法兰坐标系做变换
    Eigen::AngleAxisd rollAngle(tcp_offset_roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(tcp_offset_pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(tcp_offset_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q(yawAngle * pitchAngle * rollAngle);
    Eigen::Matrix3d tcp_offset_rotation = q.toRotationMatrix();
    Eigen::Vector3d tcp_offset_translation(tcp_offset_x, tcp_offset_y, tcp_offset_z);
    Eigen::Matrix4d tcp_offset_transform = Eigen::Matrix4d::Identity();
    tcp_offset_transform.block<3, 3>(0, 0) = tcp_offset_rotation;
    tcp_offset_transform.block<3, 1>(0, 3) = tcp_offset_translation;
    Eigen::Matrix4d T6 = computeTransformationMatrix(dh_parameters[5], joint_positions[5]);
    T6 = T6 * tcp_offset_transform;

    // 整体的再乘 T6
    T = T * T6;

    return T;
}

void RtsiReceive::updateJointPosition()
{
    while (m_running)
    {
        Rtsi::DataRecipePtr &recipe = rt->getOutputDataToRecipe();
        if (recipe->getID() == out_recipe2->getID())
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            std::vector<double> new_joint_positions;
            for (size_t i = 0; i < 6; i++)
            {
                new_joint_positions.push_back((*recipe)["actual_joint_positions"].value.v6d[i]);
            }

            if (new_joint_positions != m_last_joint_positions)
            {
                m_last_joint_positions = new_joint_positions;
                // 触发回调函数
                if (event_driven_joint_update_callback)
                {
                    event_driven_joint_update_callback(m_last_joint_positions);
                }
            }

            if (real_time_joint_update_callback)
            {
                real_time_joint_update_callback(m_last_joint_positions);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(int(100 / m_jointRate))); // 频率是更新频率的10倍
    }
}

std::vector<double> RtsiReceive::matrixToVector(const Eigen::Matrix4d &matrix)
{
    return {matrix(0, 3), matrix(1, 3), matrix(2, 3),
            std::atan2(matrix(2, 1), matrix(2, 2)),                                                          // roll
            std::atan2(-matrix(2, 0), std::sqrt(matrix(2, 1) * matrix(2, 1) + matrix(2, 2) * matrix(2, 2))), // pitch
            std::atan2(matrix(1, 0), matrix(0, 0))};
}

Eigen::Matrix4d RtsiReceive::computeTransformationMatrix(const DHParameters &dh_parameters, double theta)
{
    // 构建变换矩阵
    Eigen::Matrix4d T;
    T << cos(theta), -sin(theta), 0, dh_parameters.a,
        sin(theta) * cos(dh_parameters.alpha), cos(theta) * cos(dh_parameters.alpha), -sin(dh_parameters.alpha), -dh_parameters.d * sin(dh_parameters.alpha),
        sin(theta) * sin(dh_parameters.alpha), cos(theta) * sin(dh_parameters.alpha), cos(dh_parameters.alpha), dh_parameters.d * cos(dh_parameters.alpha),
        0, 0, 0, 1;
    return T;
}