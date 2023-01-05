#include "ros_sockets/feedback_server.hpp"

#include "ros_sockets/nlohmann/json.hpp"

namespace communication
{

FeedbackServer::FeedbackServer(std::uint16_t port)
    : Server(port)
{}

void FeedbackServer::processInboundJson(nlohmann::json json_data)
{
  if (!json_data.contains("action"))
    return;
  if (json_data["action"] != "get_feedback_data")
    return;
}

std::string FeedbackServer::json_datastring()
{
  nlohmann::json j;
  j["position"] = {pose_.pose.position.x,
                    pose_.pose.position.y,
                    pose_.pose.position.z};
  j["orientation"] = {pose_.pose.orientation.w,
                      pose_.pose.orientation.x,
                      pose_.pose.orientation.y,
                      pose_.pose.orientation.z};
  j["linear_vel"] = {twist_.twist.linear.x,
                      twist_.twist.linear.y,
                      twist_.twist.linear.z};
  j["angular_vel"] = {twist_.twist.angular.x,
                      twist_.twist.angular.y,
                      twist_.twist.angular.z};
  return j.dump();
}

} // namespace communication
