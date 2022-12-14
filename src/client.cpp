#include "client.hpp"
#include "ros/ros.h"

namespace communication
{

Client::Client() : socket_(io_service_)
{
  
  //connection
  // ROS_INFO_STREAM("Connecting...");
  socket_.connect( asio::ip::tcp::endpoint( asio::ip::address::from_string("192.168.1.207"), 42422 ));
  // ROS_INFO_STREAM("Connected");
  // request/message from client
  std::string msg = "Hello from Client!\n";
  boost::system::error_code error;
  asio::write( socket_, asio::buffer(msg), error );
  if( !error ) {
    ROS_INFO_STREAM("Client sent message");
  }
  else {
    ROS_INFO_STREAM("Send failed");
  }
}

} // namespace client