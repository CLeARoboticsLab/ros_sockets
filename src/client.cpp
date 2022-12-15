#include "client.hpp"
#include "ros/ros.h"

namespace communication
{

Client::Client(std::string ip, int port) 
    : socket_(io_service_),
      endpoint_(asio::ip::address::from_string(ip), port)
{
  socket_.open(asio::ip::udp::v4());  
}

Client::~Client()
{
  socket_.close();
  io_service_.stop();
}

void Client::send_data(std::string data)
{
  boost::system::error_code err;
  auto sent = socket_.send_to(asio::buffer(data), endpoint_, 0, err);
  ROS_INFO_STREAM("Client sent data");
}

} // namespace client