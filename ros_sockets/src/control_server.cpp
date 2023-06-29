#include "ros_sockets/control_server.hpp"

#include "ros_sockets/nlohmann/json.hpp"
#include "ros_sockets/control_data.hpp"

namespace communication
{

ControlServer::ControlServer(std::shared_ptr<ControlData> control_data, std::uint16_t port)
    : Server(port),
      control_data_(std::move(control_data))
{}

void ControlServer::processInboundJson(nlohmann::json json_data)
{
  std::vector<VelocityCommand> commands;
  for (const auto &action : json_data["controls"])
  {
    VelocityCommand command;
    command.x = action[0];
    command.phi = action[1];
    commands.push_back(command);
  }
  control_data_->update(commands);
}

} // namespace communication
