#pragma once

#include "nlohmann/json.hpp"
#include "server.hpp"
#include "control_data.hpp"

namespace communication
{

class ControlServer : public Server
{
  public:
    ControlServer(std::shared_ptr<ControlData> control_data, std::uint16_t port);

  private:
    void processJson(nlohmann::json json_data);

    std::shared_ptr<ControlData> control_data_;
};

} // namespace communication
