#include <string>

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include "ros_sockets/client.hpp"
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

      pose_sub_ = nh_.subscribe(
          "/" + tracker_name + "/pose", 100,
          &StateFeedback::pose_callback, this);
      twist_sub_ = nh_.subscribe(
          "/" + tracker_name + "/twist", 100,
          &StateFeedback::twist_callback, this); 
      timer_ = nh_.createTimer(
          ros::Duration(1.0/max_update_frequency),
          &StateFeedback::timerCallback, this);
      client_ = new communication::Client(ip,port);
    }

  private:

    // Used to limit the rate that pose data is sent
    bool ready_to_send_;

    // State data
    geometry_msgs::PoseStamped pose_stamped_;
    geometry_msgs::TwistStamped twist_stamped_;

    // ROS objects
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber twist_sub_;
    ros::Timer timer_;

    communication::Client *client_;

    // When PoseStamped data is published, send the state data over UDP. Note:
    // PoseStamped data is updated via this callback; the other state data that
    // is sent is simply the data when it was last updated by via its callback.
    void pose_callback(geometry_msgs::PoseStamped pose_stamped)
    {
      if (ready_to_send_)
      {
        ready_to_send_ = false;
        pose_stamped_ = pose_stamped;
        client_->sendData(json_datastring());
        ROS_INFO_STREAM("State data sent");
      }
    }

    // Update when TwistStamped data is published, update its stored data. 
    void twist_callback(geometry_msgs::TwistStamped twist_stamped)
    {
      twist_stamped_ = twist_stamped;
    }

    // When the timer triggers, set the object ready to send. 
    void timerCallback(const ros::TimerEvent&)
    {
      ready_to_send_ = true;
    }

    std::string json_datastring()
    {
      json j;
      j["position"] = {pose_stamped_.pose.position.x,
                        pose_stamped_.pose.position.y,
                        pose_stamped_.pose.position.z};
      j["orientation"] = {pose_stamped_.pose.orientation.w,
                          pose_stamped_.pose.orientation.x,
                          pose_stamped_.pose.orientation.y,
                          pose_stamped_.pose.orientation.z};
      j["linear_vel"] = {twist_stamped_.twist.linear.x,
                          twist_stamped_.twist.linear.y,
                          twist_stamped_.twist.linear.z};
      j["angular_vel"] = {twist_stamped_.twist.angular.x,
                          twist_stamped_.twist.angular.y,
                          twist_stamped_.twist.angular.z};
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