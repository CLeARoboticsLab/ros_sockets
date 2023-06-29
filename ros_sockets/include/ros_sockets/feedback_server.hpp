#pragma once

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include "nlohmann/json.hpp"
#include "server.hpp"

namespace communication
{

class FeedbackServer : public Server
{
  public:
    FeedbackServer(std::uint16_t port);
    void updatePose(geometry_msgs::PoseStamped pose){pose_ = pose;}
    void updateTwist(geometry_msgs::TwistStamped twist){twist_ = twist;}

  private:
    void processInboundJson(nlohmann::json json_data);
    std::string json_datastring();
    geometry_msgs::PoseStamped pose_;
    geometry_msgs::TwistStamped twist_;
};

} // namespace communication
