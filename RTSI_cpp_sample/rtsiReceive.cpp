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