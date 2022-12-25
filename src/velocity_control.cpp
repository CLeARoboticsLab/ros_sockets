#include <iostream>
#include <signal.h>

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

#include "ros_sockets/server.hpp"
#include "ros_sockets/control_data.hpp"

// Flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

void publishCommand(ros::Publisher publisher, VelocityCommand command)
{
  geometry_msgs::Twist twist;
  twist.linear.x = command.x;
  twist.angular.z = command.phi;
  publisher.publish(twist);
}

// Handles ctrl+c of the node
void sigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

auto main(int argc, char **argv) -> int
{
  auto control_data = std::make_shared<ControlData>();

  // initialize node
  ros::init(argc, argv, "velocity_control", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigIntHandler);
  ros::NodeHandle nh;
  double timestep;
  nh.getParam("/velocity_control/timestep", timestep);
  
  // check for a valid timestep
  if (timestep <= 0.0)
  {
    ROS_ERROR_STREAM("Time step must be greater than 0.0. Shutting down.");
    ros::shutdown();
    return 0;
  }

  // continue node initialization
  ROS_INFO_STREAM("Initializing node with timestep of " << timestep << " sec");
  const auto queue_size = 100;
  ros::Publisher publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", queue_size);
  ros::Rate loop_rate(1 / timestep);

  // start server
  int port;
  nh.getParam("/velocity_control/port", port);
  ROS_INFO_STREAM("Starting communication server on port " << port);
  communication::Server server(control_data, port);

  while (!g_request_shutdown && ros::ok())
  {
    // if getNewData() returns an emtpy vector, then no data has appeared within 
    // the timeout period
    auto commands = control_data->getNewData();
    if (commands.empty())
      continue;

    // append a stop command to the end of any incoming control sequence
    // to ensure robot stops after all commands are sent
    VelocityCommand stop_command;
    commands.push_back(stop_command);

    // pushlish velocity commands in a timed loop
    for (auto command : commands)
    {
      if (control_data->hasNewData())
        break;
      publishCommand(publisher, command);
      ROS_DEBUG_STREAM("Velocity command x: " << command.x << "  phi: " << command.phi);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  // shutdown gracefully if node is interrupted
  ROS_INFO_STREAM("Stopping and exiting..." );
  VelocityCommand stop_command;
  publishCommand(publisher, stop_command);
  ros::shutdown();
  std::cout << "Exited." << std::endl;

  return 0;
}