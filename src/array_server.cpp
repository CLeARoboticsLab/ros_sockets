#include "ros_sockets/array_server.hpp"

#include "ros_sockets/nlohmann/json.hpp"

namespace communication
{

ArrayServer::ArrayServer(std::shared_ptr<CommandData> command_data, std::uint16_t port)
    : Server(port),
      command_data_(std::move(command_data))
{}

void ArrayServer::processInboundJson(nlohmann::json json_data)
{
  // TODO
}

} // namespace communication