#pragma once

#include <string>
#include <boost/asio.hpp>

namespace communication
{

namespace asio = boost::asio;

class Client
{
  public:
    Client(std::string ip, int port);
    ~Client();
    void sendData(std::string data);
  private:
    asio::io_service io_service_;
    asio::ip::udp::socket socket_;
    asio::ip::udp::endpoint endpoint_; 
};

} // namespace client