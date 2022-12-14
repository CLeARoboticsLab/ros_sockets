#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <string>
#include "client.hpp"

class StateFeedback
{
  public:

    StateFeedback()
    {
      std::string tracker_name;
      nh_.getParam("/state_feedback/tracker_name", tracker_name);
      double max_update_frequency;
      nh_.getParam("/state_feedback/max_update_frequency", max_update_frequency);

      std::string sub_topic = "/" + tracker_name + "/pose";
      sub_ = nh_.subscribe(
          sub_topic, 100,
          &StateFeedback::pose_callback, this);  
      timer_ = nh_.createTimer(
          ros::Duration(1.0/max_update_frequency),
          &StateFeedback::timerCallback, this);
    }

  private:

    // Used to limit the rate that pose data is sent
    bool ready_to_send_;

    // ROS objects
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Timer timer_;

    void pose_callback(geometry_msgs::PoseStamped pose_stamped)
    {
      if (ready_to_send_)
      {
        ready_to_send_ = false;
        ROS_INFO_STREAM(pose_stamped);
      }
    }

    // When the timer triggers, set the object ready to send. 
    void timerCallback(const ros::TimerEvent&)
    {
      ready_to_send_ = true;
    }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "state_feedback");

  StateFeedback state_feedback;

  communication::Client client;

  ros::spin();
  
  return 0;
}