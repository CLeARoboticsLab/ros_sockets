#include "communication.hpp"
#include "control_data.hpp"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <iostream>
#include <signal.h>

// Flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

void publish_command(ros::Publisher publisher, VelocityCommand command) {
	geometry_msgs::Twist twist;
	twist.linear.x = command.x;
	twist.angular.z = command.phi;
	publisher.publish(twist);
}

// Handles ctrl+c of the node
void sig_int_handler(int sig) {
	g_request_shutdown = 1;
}

auto main(int argc, char **argv) -> int {
	auto control_data = std::make_shared<ControlData>();

	// initialize node
	ros::init(argc, argv, "velocity_control", ros::init_options::NoSigintHandler);
	signal(SIGINT, sig_int_handler);
	ros::NodeHandle nh;
	double TIMESTEP;
	nh.getParam("/velocity_control/TIMESTEP", TIMESTEP);
	
	// check for a valid timestep
	if (TIMESTEP <= 0.0) {
		ROS_ERROR_STREAM("Time step must be greater than 0.0. Shutting down.");
		ros::shutdown();
  		return 0;
	}

	// continue node initialization
	ROS_INFO_STREAM("Initializing node with timestep of " << TIMESTEP << " sec");
	const auto queue_size = 100;
	ros::Publisher publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", queue_size);
	ros::Rate loop_rate(1 / TIMESTEP);

	// start server
	int PORT;
	nh.getParam("/velocity_control/PORT", PORT);
	ROS_INFO_STREAM("Starting communication server on port " << PORT);
	Communication::Server server(control_data, PORT);

	while (!g_request_shutdown && ros::ok()) {
		
		// if get_new_data() returns an emtpy vector, then no data has appeared within 
		// the timeout period
		auto commands = control_data->get_new_data();
		if (commands.empty()) {
			continue;
		}

		// append a stop command to the end of any incoming control sequence
		// to ensure robot stops after all commands are sent
		VelocityCommand stop_command;
		commands.push_back(stop_command);

		// pushlish velocity commands in a timed loop
		for (auto command : commands) {
			if (control_data->has_new_data()) {
				break;
			}
			publish_command(publisher, command);
			ROS_DEBUG_STREAM("Velocity command x: " << command.x << "  phi: " << command.phi);
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

	// shutdown gracefully if node is interrupted
	ROS_INFO_STREAM("Stopping and exiting..." );
	VelocityCommand stop_command;
	publish_command(publisher, stop_command);
	ros::shutdown();
	std::cout << "Exited." << std::endl;

  	return 0;
}
