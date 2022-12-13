#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <gazebo_msgs/ModelStates.h>
#include <string>

bool ready_to_pub = false;
geometry_msgs::Pose model_pose;
std::string model_name;
int model_idx = -1;

int getModelIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

void model_states_callback(gazebo_msgs::ModelStates model_states)
{
    model_idx = getModelIndex(model_states.name, model_name);
    if (!ready_to_pub || model_idx == -1) {
        return;
    }
    ready_to_pub = false;
    model_pose = model_states.pose[model_idx];
    ROS_INFO_STREAM(model_pose); //TODO: debug stream
}

void timerCallback(const ros::TimerEvent&) {
    ready_to_pub = true;
} 

int main(int argc, char** argv) {

    ros::init(argc, argv, "gazebo_tracker");
    ros::NodeHandle nh;
    
    double update_frequency;
    nh.getParam("/gazebo_tracker/update_frequency", update_frequency);
    nh.getParam("/gazebo_tracker/model_name", model_name);

    ros::Timer timer = nh.createTimer(ros::Duration(1.0/update_frequency), timerCallback);
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 100, model_states_callback);
    // TODO: publisher

    ros::spin();
    
    return 0;
}