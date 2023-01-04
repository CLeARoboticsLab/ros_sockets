#pragma once

#include <boost/asio.hpp>
#include <thread>

#include "server.hpp"
#include "control_data.hpp"

namespace communication
{

namespace asio = boost::asio;

class ControlServer : public Server
{
  public:
    ControlServer(std::shared_ptr<ControlData> control_data, std::uint16_t port);
  //   Server(std::shared_ptr<ControlData> control_data, std::uint16_t port);
  //   Server(const Server &) = delete;
  //   Server(Server &&) = delete;
  //   auto operator=(const Server &) -> Server & = delete;
  //   auto operator=(Server &&) -> Server & = delete;
  //   ~Server();

  // private:
  //   void readHandler(const boost::system::error_code &error,
  //       std::size_t bytes_transferred);
  //   void acceptHandler(const boost::system::error_code &error);
  //   void scheduleRead();
  //   void scheduleAccept();

  //   std::shared_ptr<ControlData> control_data_;
  //   std::thread thread_;
  //   asio::io_service io_service_;
  //   asio::ip::tcp::acceptor acceptor_;
  //   asio::ip::tcp::socket socket_;
  //   asio::streambuf buffer_;
};

} // namespace communication
