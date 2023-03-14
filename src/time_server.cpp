#include "ros_sockets/time_server.hpp"

#include "ros_sockets/nlohmann/json.hpp"

namespace communication
{

TimeServer::TimeServer(std::shared_ptr<CommandData> command_data, std::uint16_t port)
    : Server(port),
      command_data_(std::move(command_data))
{}

void TimeServer::processInboundJson(nlohmann::json json_data)
{
  if (!json_data.contains("action"))
    return;

  if (json_data["action"] == "start_experiment")
  {
    time_ = ros::Time::now();
    command_data_->update();
  }
  else if (json_data["action"] == "stop_experiment")
  {
    ros::Time t;
    time_ = t;
    command_data_->update();
  }
  else if (json_data["action"] == "get_time_elapsed")
  {
    scheduleWrite(json_datastring()+'\n');
  }
}

std::string TimeServer::json_datastring()
{
  ros::Duration d = ros::Time::now() - time_;
  nlohmann::json j;
  j["elapsed_time"] = d.toSec();
  return j.dump();
}

} // namespace communication