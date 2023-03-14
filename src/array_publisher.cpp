#include <signal.h>
#include <string>

#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"

#include "ros_sockets/array_server.hpp"
#include "ros_sockets/command_data.hpp"

// Flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Handles ctrl+c of the node
void sigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

class ArrayPublisher
{
  public:

    ArrayPublisher(std::shared_ptr<CommandData> command_data)
        : command_data_(std::move(command_data))
      {
        int port;
        nh_.getParam("array_publisher/port", port);
        nh_.getParam("array_publisher/topic_name", topic_name_);

        ROS_INFO_STREAM("Initializing Arrary Publisher node");
        const auto queue_size = 100;
        pub_ = nh_.advertise<std_msgs::Float64MultiArray>(topic_name_, queue_size);

        ROS_INFO_STREAM("Starting array server on port " << port);
        array_server_ = new communication::ArrayServer(command_data_, port);
      }

    ~ArrayPublisher()
    {
      delete array_server_;
    }

    void getData()
    {
      // if getNewData() returns false, then no data has appeared within the
      // timeout period
      if (!command_data_->getNewData())
        return;

      std_msgs::Float64MultiArray array_msg;
      // time_msg.data = time_server_->time(); TODO
      pub_.publish(array_msg);
      ROS_INFO_STREAM("Publishing array");
    }

    void shutdown()
    {
      ros::shutdown();
    }

  private:
    std::string topic_name_;
    ros::NodeHandle nh_;
    ros::Publisher pub_;

    std::shared_ptr<CommandData> command_data_;
    communication::ArrayServer *array_server_;
};

auto main(int argc, char **argv) -> int
{
  // initialize node
  ros::init(argc, argv, "array_publisher", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigIntHandler);

  ArrayPublisher array_publisher(std::make_shared<CommandData>());

  while (!g_request_shutdown && ros::ok())
  {
    array_publisher.getData();
  }

  // shutdown gracefully if node is interrupted
  array_publisher.shutdown();

  return 0;
}