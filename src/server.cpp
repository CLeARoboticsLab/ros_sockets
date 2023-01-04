#include "ros_sockets/server.hpp"

#include <iostream>
#include <utility>

#include <ros/ros.h>
#include "ros_sockets/nlohmann/json.hpp"

namespace communication
{

Server::Server(std::uint16_t port)
    : acceptor_(io_service_, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port)),
      socket_(io_service_)
{
  scheduleAccept();
  thread_ = std::thread([this]() { io_service_.run(); });
}

Server::~Server()
{
  if (socket_.is_open())
    socket_.close();
  io_service_.stop();
  if (thread_.joinable())
    thread_.join();
}

void Server::acceptHandler(const boost::system::error_code &error)
{
  if (error)
  {
    ROS_ERROR_STREAM("Failed to accept: " << error);
    scheduleAccept();
  }
  else
  {
    ROS_INFO_STREAM("Client connected");
    scheduleRead();
  }
}

void Server::scheduleRead()
{
  asio::async_read_until(socket_, buffer_, "\n",
      [this](const boost::system::error_code &error, std::size_t bytes_transferred)
      {
        this->readHandler(error, bytes_transferred);
      });
}

void Server::scheduleAccept()
{
	ROS_INFO_STREAM("Waiting for client to connect...");
  acceptor_.async_accept(socket_, 
      [this](const boost::system::error_code &error)
      {
        this->acceptHandler(error);
      });
}

void Server::readHandler(const boost::system::error_code &error,
    std::size_t /*bytes_transferred*/)
{
  if (error)
  {
    ROS_WARN_STREAM("Failed to read from socket: " << error.message());
    ROS_INFO_STREAM("Closing socket...");
    socket_.close();
    scheduleAccept();
  }
  else
  {
    std::istream stream(&buffer_);
    std::string payload;
    std::getline(stream, payload);
    nlohmann::json json_data = nlohmann::json::parse(payload);
    processJson(json_data);
    scheduleRead();
  }
}

} // namespace communication
