#pragma once

#include <boost/asio.hpp>
#include <thread>

#include "nlohmann/json.hpp"

namespace communication
{

namespace asio = boost::asio;

class Server
{
  public:
    Server(std::uint16_t port);
    Server(const Server &) = delete;
    Server(Server &&) = delete;
    auto operator=(const Server &) -> Server & = delete;
    auto operator=(Server &&) -> Server & = delete;
    ~Server();

  private:
    void readHandler(const boost::system::error_code &error,
        std::size_t bytes_transferred);
    void acceptHandler(const boost::system::error_code &error);
    void scheduleRead();
    void scheduleAccept();
    virtual void processJson(nlohmann::json json_data){};

    std::thread thread_;
    asio::io_service io_service_;
    asio::ip::tcp::acceptor acceptor_;
    asio::ip::tcp::socket socket_;
    asio::streambuf buffer_;
};

} // namespace communication
