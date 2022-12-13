#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <gazebo_msgs/ModelStates.h>

bool ready_to_pub = false;
geometry_msgs::Pose model_pose;

void model_states_callback(gazebo_msgs::ModelStates model_states)
{
    if (!ready_to_pub) {
        return;
    }
    ready_to_pub = false;
    model_pose = model_states.pose[1]; // TODO: smarter model selection
    ROS_INFO_STREAM(model_pose); // TODO: publisher
}

void timerCallback(const ros::TimerEvent&) {
    ready_to_pub = true;
} 

int main(int argc, char** argv) {

    ros::init(argc, argv, "gazebo_pose");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 100, model_states_callback);
    double update_frequency = 0.5; // TODO: make parameter
    ros::Timer timer = nh.createTimer(ros::Duration(1.0/update_frequency), timerCallback);

    ros::spin();
    
    return 0;
}