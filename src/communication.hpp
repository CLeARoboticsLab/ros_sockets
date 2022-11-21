#pragma once

#include "control_data.hpp"
#include <boost/asio.hpp>
#include <thread>

namespace Communication {

namespace asio = boost::asio;

struct Server {
	public:
		Server(std::shared_ptr<ControlData> control_data, std::uint16_t port);
		Server(const Server &) = delete;
		Server(Server &&) = delete;
		auto operator=(const Server &) -> Server & = delete;
		auto operator=(Server &&) -> Server & = delete;
		~Server();

	private:
		void read_handler(const boost::system::error_code &error,
							std::size_t bytes_transferred);
		void accept_handler(const boost::system::error_code &error);
		void schedule_read();
		void schedule_accept();

		std::shared_ptr<ControlData> control_data;
		std::thread thread;
		asio::io_service io_service;
		asio::ip::tcp::acceptor acceptor;
		asio::ip::tcp::socket socket;
		asio::streambuf buffer;
};

} // namespace Communication
