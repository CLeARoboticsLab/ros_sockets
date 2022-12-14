#include "client.hpp"
#include "ros/ros.h"

namespace communication
{

Client::Client() : socket_(io_service_)
{
  asio::ip::udp::endpoint remote_endpoint = 
      asio::ip::udp::endpoint(asio::ip::address::from_string("192.168.1.207"), 42422);
  socket_.open(asio::ip::udp::v4());

  std::string payload = "Data sent via UDP";
  boost::system::error_code err;
  auto sent = socket_.send_to(asio::buffer(payload), remote_endpoint, 0, err);
  socket_.close();

  ROS_INFO_STREAM("Client sent message");
}

} // namespace client