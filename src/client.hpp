#pragma once

#include <boost/asio.hpp>

namespace communication
{

namespace asio = boost::asio;

class Client
{
  public:
    Client();
  private:
    asio::io_service io_service_;
    asio::ip::udp::socket socket_;
};

} // namespace client