#include <signal.h>

#include <ros/ros.h>
#include "std_msgs/Time.h"

#include "ros_sockets/time_server.hpp"
#include "ros_sockets/command_data.hpp"

// Flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Handles ctrl+c of the node
void sigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

class ExperimentTime
{
  public:

    ExperimentTime(std::shared_ptr<CommandData> command_data)
        : command_data_(std::move(command_data))
      {
        int port;
        nh_.getParam("experiment_time/port", port);

        ROS_INFO_STREAM("Initializing Experiment Time node");
        const auto queue_size = 10;
        pub_ = nh_.advertise<std_msgs::Time>("start_time", queue_size);

        ROS_INFO_STREAM("Starting time server on port " << port);
        time_server_ = new communication::TimeServer(command_data_, port);
      }

    ~ExperimentTime()
    {
      delete time_server_;
    }

    void getData()
    {
      // if getNewData() returns false, then no data has appeared within the
      // timeout period
      if (!command_data_->getNewData())
        return;

      std_msgs::Time time_msg;
      time_msg.data = time_server_->time();
      pub_.publish(time_msg);
      ROS_INFO_STREAM("Publishing start time of: " << time_msg.data);
    }

    void shutdown()
    {
      std_msgs::Time time_msg;
      ros::Time t;
      time_msg.data = t;
      pub_.publish(time_msg);
      ros::shutdown();
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;

    std::shared_ptr<CommandData> command_data_;
    communication::TimeServer *time_server_;
};

auto main(int argc, char **argv) -> int
{
  // initialize node
  ros::init(argc, argv, "experiment_time", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigIntHandler);

  ExperimentTime experiment_time(std::make_shared<CommandData>());

  while (!g_request_shutdown && ros::ok())
  {
    experiment_time.getData();
  }

  // shutdown gracefully if node is interrupted
  experiment_time.shutdown();

  return 0;
}