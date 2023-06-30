#pragma once

#include <ros/ros.h>

#include "nlohmann/json.hpp"
#include "server.hpp"
#include "command_data.hpp"

namespace communication
{

class TimeServer : public Server
{
  public:
    TimeServer(std::shared_ptr<CommandData> command_data, std::uint16_t port);
    ros::Time time(){return time_;}

  private:
    void processInboundJson(nlohmann::json json_data);
    std::string json_datastring();
    std::shared_ptr<CommandData> command_data_;
    ros::Time time_;
};

} // namespace communication