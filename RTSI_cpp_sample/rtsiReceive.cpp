#include "rtsiReceive.h"

RtsiReceive::RtsiReceive(const std::string &ip, int jointRate, int payloadRate)
{
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

    setTCPOffset(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

RtsiReceive::~RtsiReceive()
{
    RtsiStop();
}

void RtsiReceive::RtsiStart()
{
    rt->start();
}

void RtsiReceive::RtsiStop()
{
    rt->pause();
    rt->disconnect();
}

std::vector<double> RtsiReceive::GetActualJointPositions()
{
    Rtsi::DataRecipePtr &recipe = rt->getOutputDataToRecipe();
    if (recipe->getID() == out_recipe2->getID())
    {
        std::vector<double> joint_positions;
        for (size_t i = 0; i < 6; i++)
        {
            joint_positions.push_back((*recipe)["actual_joint_positions"].value.v6d[i]);
        }
        return joint_positions;
    }
    return {};
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
    Eigen::Matrix4d tcp_matrix = getTcpTransformationMatrix(joint_positions);
    return {tcp_matrix(0, 3), tcp_matrix(1, 3), tcp_matrix(2, 3),
            std::atan2(tcp_matrix(2, 1), tcp_matrix(2, 2)),                                                                      // roll
            std::atan2(-tcp_matrix(2, 0), std::sqrt(tcp_matrix(2, 1) * tcp_matrix(2, 1) + tcp_matrix(2, 2) * tcp_matrix(2, 2))), // pitch
            std::atan2(tcp_matrix(1, 0), tcp_matrix(0, 0))};                                                                     // yaw
}

Eigen::Matrix4d RtsiReceive::forwardKinematics(const std::vector<double> &joint_positions)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < joint_positions.size(); i++)
    {
        T = T * computeTransformationMatrix(dh_parameters[i], joint_positions[i]);
    }
    return T;
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

Eigen::Matrix4d RtsiReceive::getTcpTransformationMatrix(const std::vector<double> &joint_positions)
{
    // 计算正向运动学
    Eigen::Matrix4d T = forwardKinematics(joint_positions);

    // TODO：TCP带有偏移情况怎么处理

    return T;
}