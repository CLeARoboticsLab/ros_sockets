#pragma once

#include <ros/ros.h>

#include "nlohmann/json.hpp"
#include "server.hpp"
#include "command_data.hpp"

namespace communication
{

class ArrayServer : public Server
{
  public:
    ArrayServer(std::shared_ptr<CommandData> command_data, std::uint16_t port);

  private:
    void processInboundJson(nlohmann::json json_data);
    std::shared_ptr<CommandData> command_data_;
};

} // namespace communication