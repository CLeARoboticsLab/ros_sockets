#pragma once

#include "nlohmann/json.hpp"
#include "server.hpp"

#include "ros_sockets_msgs/RolloutData.h"

namespace communication
{

class RolloutDataServer : public Server
{
  public:
    RolloutDataServer(std::uint16_t port);
    void update(ros_sockets_msgs::RolloutData rollout_data_msg){rollout_data_ = rollout_data_msg;}

  private:
    void processInboundJson(nlohmann::json json_data);
    std::string json_datastring();
    ros_sockets_msgs::RolloutData rollout_data_;
};

} // namespace communication