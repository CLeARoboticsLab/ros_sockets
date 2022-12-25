#include <string>

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

// Tracks the pose and twist of a model in Gazebo, <model_name> and publishes
// them to <tracker_name>/pose and <tracker_name>/twist.
class Tracker 
{
  public:

    Tracker()
    {
      model_idx_ = -1;
      ready_to_pub_ = false;

      nh_.getParam("/gazebo_tracker/tracker_name", tracker_name_);
      nh_.getParam("/gazebo_tracker/model_name", model_name_);
      double update_frequency;
      nh_.getParam("/gazebo_tracker/update_frequency", update_frequency);
      
      pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
          "/" + tracker_name_ + "/pose", 
          100);
      twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
          "/" + tracker_name_ + "/twist", 
          100);
      sub_ = nh_.subscribe(
          "/gazebo/model_states", 100,
          &Tracker::modelStatesCallback, this);
      timer_ = nh_.createTimer(
          ros::Duration(1.0/update_frequency),
          &Tracker::timerCallback, this);
    }

  private:

    // General information
    std::string tracker_name_;
    std::string model_name_;
    int model_idx_;
    
    // Pose data
    geometry_msgs::Pose model_pose_;
    geometry_msgs::PoseStamped model_pose_stamped_;

    // Twist data
    geometry_msgs::Twist model_twist_;
    geometry_msgs::TwistStamped model_twist_stamped_;

    // Used to control the rate that pose data is published
    bool ready_to_pub_;

    // ROS objects
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::Publisher twist_pub_;
    ros::Subscriber sub_;
    ros::Timer timer_;

    // Returns the index of string s in vector v
    int getModelIndex(std::vector<std::string> v, std::string s)
    {
      for(int i = 0; i < v.size(); i++)
      {
        if(v[i].compare(s) == 0)
          return i;
      }
      return -1;
    }

    // Updates the model index and publishes state data if ready to publish.
    void modelStatesCallback(gazebo_msgs::ModelStates model_states)
    {
      model_idx_ = getModelIndex(model_states.name, model_name_);
      if (ready_to_pub_ && model_idx_ != -1)
      {
        model_pose_ = model_states.pose[model_idx_];
        model_twist_ = model_states.twist[model_idx_];
        publishState();
      }
    }
    
    // When the timer triggers, set the object ready to publish. 
    void timerCallback(const ros::TimerEvent&)
    {
      ready_to_pub_ = true;
    }

    // Publishes the pose data to the topic.
    void publishState()
    {
      ready_to_pub_ = false;

      model_twist_stamped_.twist = model_twist_;
      model_pose_stamped_.pose = model_pose_;
      
      twist_pub_.publish(model_twist_stamped_);
      pose_pub_.publish(model_pose_stamped_);
    } 

};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "gazebo_tracker");

  Tracker tracker;

  ros::spin();
  
  return 0;
}