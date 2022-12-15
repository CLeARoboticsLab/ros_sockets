#include "server.hpp"
#include "nlohmann/json.hpp"
#include "ros/ros.h"
#include <iostream>
#include <utility>

namespace communication {

Server::Server(std::shared_ptr<ControlData> control_data, std::uint16_t port)
    		: control_data(std::move(control_data)),
    		acceptor(io_service, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port)),
    		socket(io_service) {
  	schedule_accept();
  	thread = std::thread([this]() { io_service.run(); });
}

Server::~Server() {
  	if (socket.is_open()) {
    	socket.close();
  	}
  	io_service.stop();
  	if (thread.joinable()) {
    	thread.join();
  	}
}

void Server::accept_handler(const boost::system::error_code &error) {
  	if (error) {
		ROS_ERROR_STREAM("Failed to accept: " << error);
    	schedule_accept();
  	} else {
		ROS_INFO_STREAM("Client connected");
    	schedule_read();
  	}
}

void Server::schedule_read() {
  	asio::async_read_until(socket, buffer, "\n",
                         	[this](const boost::system::error_code &error,
                                std::size_t bytes_transferred) {
                           	this->read_handler(error, bytes_transferred);
                         	});
}

void Server::schedule_accept() {
	ROS_INFO_STREAM("Waiting for client to connect...");
  	acceptor.async_accept(socket, [this](const boost::system::error_code &error) {
    	this->accept_handler(error);
  	});
}

void Server::read_handler(const boost::system::error_code &error,
                          std::size_t /*bytes_transferred*/) {
  	if (error) {
		ROS_WARN_STREAM("Failed to read from socket: " << error.message());
		ROS_INFO_STREAM("Closing socket...");
		socket.close();
		schedule_accept();
  	} else {
		std::istream stream(&buffer);
		std::string payload;
		std::getline(stream, payload);
		auto object = nlohmann::json::parse(payload);
		std::vector<VelocityCommand> commands;
    	for (const auto &action : object["controls"]) {
      		VelocityCommand command;
			command.x = action[0];
			command.phi = action[1];
			commands.push_back(command);
    	}
    	control_data->update(commands);
    	schedule_read();
  	}
}

} // namespace communication
