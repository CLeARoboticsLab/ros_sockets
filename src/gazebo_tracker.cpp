#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <gazebo_msgs/ModelStates.h>
#include <string>

// Tracks the pose of a model in Gazebo, <model_name> and publishes it to
// /<tracker_name>/pose.
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
      
      std::string pub_topic = "/" + tracker_name_ + "/pose";
      pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
          pub_topic, 100);
      sub_ = nh_.subscribe(
          "/gazebo/model_states", 100,
          &Tracker::model_states_callback, this);
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

    // Used to control the rate that pose data is published
    bool ready_to_pub_;

    // ROS objects
    ros::NodeHandle nh_;
    ros::Publisher pub_;
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

    // Updates the model index and publishes pose data if ready to publish.
    void model_states_callback(gazebo_msgs::ModelStates model_states)
    {
      model_idx_ = getModelIndex(model_states.name, model_name_);
      if (ready_to_pub_ && model_idx_ != -1)
      {
        model_pose_ = model_states.pose[model_idx_];
        publish_pose_stamped();
      }
    }
    
    // When the timer triggers, set the object ready to publish. 
    void timerCallback(const ros::TimerEvent&)
    {
      ready_to_pub_ = true;
    }

    // Publishes the pose data to the topic.
    void publish_pose_stamped()
    {
      ready_to_pub_ = false;
      model_pose_stamped_.pose = model_pose_;
      pub_.publish(model_pose_stamped_);
    } 

};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "gazebo_tracker");

  Tracker tracker;

  ros::spin();
  
  return 0;
}