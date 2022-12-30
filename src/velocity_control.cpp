#include <iostream>
#include <signal.h>

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

#include "ros_sockets/server.hpp"
#include "ros_sockets/control_data.hpp"

// Flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Handles ctrl+c of the node
void sigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

class VelocityControl
{
  public:

    VelocityControl(std::shared_ptr<ControlData> control_data)
        : control_data_(std::move(control_data))
    {
      double timestep;
      nh_.getParam("velocity_control/timestep", timestep);
      check_timestep(timestep);
      int port;
      nh_.getParam("velocity_control/port", port);

      ROS_INFO_STREAM("Initializing node with timestep of " << timestep << " sec");
      const auto queue_size = 100;
      pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", queue_size);
      rate_ = new ros::Rate(1 / timestep);
      ROS_INFO_STREAM("Starting communication server on port " << port);
      server_ = new communication::Server(control_data_, port);
    }

    ~VelocityControl()
    {
      delete rate_;
      delete server_;
    }

    void getData()
    {
      // if getNewData() returns an emtpy vector, then no data has appeared
      // within the timeout period
      auto commands = control_data_->getNewData();
      if (commands.empty())
        return;

      // append a stop command to the end of any incoming control sequence
      // to ensure robot stops after all commands are sent
      VelocityCommand stop_command;
      commands.push_back(stop_command);

      // publish velocity commands in a timed loop
      for (auto command : commands)
      {
        if (control_data_->hasNewData())
          break;
        publishCommand(pub_, command);
        ROS_DEBUG_STREAM("Velocity command x: " << command.x << "  phi: " << command.phi);
        ros::spinOnce();
        rate_->sleep();
      }
    }

    void shutdown()
    {
      ROS_INFO_STREAM("Stopping and exiting..." );
      VelocityCommand stop_command;
      publishCommand(pub_, stop_command);
      ros::shutdown();
      std::cout << "Exited." << std::endl;
    }

  private:

    // ROS objects
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Rate *rate_;

    std::shared_ptr<ControlData> control_data_;
    communication::Server *server_;

    void check_timestep(double timestep)
    {
      if (timestep <= 0.0)
      {
        ROS_ERROR_STREAM("Time step must be greater than 0.0. Shutting down.");
        ros::shutdown();
      }
    }

    void publishCommand(ros::Publisher publisher, VelocityCommand command)
    {
      geometry_msgs::Twist twist;
      twist.linear.x = command.x;
      twist.angular.z = command.phi;
      publisher.publish(twist);
    }
};

auto main(int argc, char **argv) -> int
{
  // initialize node
  ros::init(argc, argv, "velocity_control", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigIntHandler);

  VelocityControl velocity_control(std::make_shared<ControlData>());

  while (!g_request_shutdown && ros::ok())
  {
    velocity_control.getData();
  }

  // shutdown gracefully if node is interrupted
  velocity_control.shutdown();

  return 0;
}