#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <string>
#include "client.hpp"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

class StateFeedback
{
  public:

    StateFeedback()
    {
      std::string tracker_name;
      nh_.getParam("/state_feedback/tracker_name", tracker_name);
      double max_update_frequency;
      nh_.getParam("/state_feedback/max_update_frequency", max_update_frequency);
      std::string ip;
      nh_.getParam("/state_feedback/ip_address", ip);
      int port;
      nh_.getParam("/state_feedback/port", port);

      std::string sub_topic = "/" + tracker_name + "/pose";
      sub_ = nh_.subscribe(
          sub_topic, 100,
          &StateFeedback::pose_callback, this);  
      timer_ = nh_.createTimer(
          ros::Duration(1.0/max_update_frequency),
          &StateFeedback::timerCallback, this);
      client_ = new communication::Client("192.168.1.207",42422);
    }

  private:

    // Used to limit the rate that pose data is sent
    bool ready_to_send_;

    // ROS objects
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Timer timer_;

    communication::Client *client_;

    void pose_callback(geometry_msgs::PoseStamped pose_stamped)
    {
      if (ready_to_send_)
      {
        ready_to_send_ = false;
        client_->send_data(json_string(pose_stamped));
        ROS_INFO_STREAM("Sent data");
      }
    }

    // When the timer triggers, set the object ready to send. 
    void timerCallback(const ros::TimerEvent&)
    {
      ready_to_send_ = true;
    }

    std::string json_string(geometry_msgs::PoseStamped pose_stamped)
    {
      json j;
      j["position"] = {pose_stamped.pose.position.x,
                        pose_stamped.pose.position.y,
                        pose_stamped.pose.position.z};
      j["orientation"] = {pose_stamped.pose.orientation.x,
                          pose_stamped.pose.orientation.y,
                          pose_stamped.pose.orientation.z,
                          pose_stamped.pose.orientation.w};
      return j.dump();
    }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "state_feedback");

  StateFeedback state_feedback;

  ros::spin();
  
  return 0;
}