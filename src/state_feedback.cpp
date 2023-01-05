#include <string>

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include "ros_sockets/feedback_server.hpp"
#include "ros_sockets/nlohmann/json.hpp"

using json = nlohmann::json;

class StateFeedback
{
  public:

    StateFeedback()
    {
      std::string tracker_name;
      nh_.getParam("state_feedback/tracker_name", tracker_name);
      int port;
      nh_.getParam("state_feedback/port", port);

      ROS_INFO_STREAM("Starting feedback server on port " << port);
      feedback_server_ = new communication::FeedbackServer(port);

      pose_sub_ = nh_.subscribe(
          "/" + tracker_name + "/pose", 100,
          &StateFeedback::pose_callback, this);
      twist_sub_ = nh_.subscribe(
          "/" + tracker_name + "/twist", 100,
          &StateFeedback::twist_callback, this); 
    }

    ~StateFeedback()
    {
      delete feedback_server_;
    }

  private:

    // ROS objects
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber twist_sub_;

    communication::FeedbackServer *feedback_server_;

    void pose_callback(geometry_msgs::PoseStamped pose)
    {
      feedback_server_->updatePose(pose);
    }

    void twist_callback(geometry_msgs::TwistStamped twist)
    {
      feedback_server_->updateTwist(twist);
    }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "state_feedback");

  StateFeedback state_feedback;

  ros::spin();
  
  return 0;
}