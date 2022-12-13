#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <gazebo_msgs/ModelStates.h>
#include <string>

class Tracker {

    public:

        Tracker() {
            model_idx_ = -1;
            ready_to_pub_ = false;

            double update_frequency;
            nh_.getParam("/gazebo_tracker/update_frequency", update_frequency);
            nh_.getParam("/gazebo_tracker/model_name", model_name_);

            timer_ = nh_.createTimer(
                ros::Duration(1.0/update_frequency),
                &Tracker::timerCallback, this);
            sub_ = nh_.subscribe(
                "/gazebo/model_states", 100,
                &Tracker::model_states_callback, this);
            // TODO: publisher
        }

    private:

        std::string model_name_;
        int model_idx_;
        bool ready_to_pub_;
        geometry_msgs::Pose model_pose_;

        ros::NodeHandle nh_;
        ros::Timer timer_;
        ros::Subscriber sub_;

        int getModelIndex(std::vector<std::string> v, std::string value) {
            for(int i = 0; i < v.size(); i++) {
                if(v[i].compare(value) == 0)
                    return i;
            }
            return -1;
        }

        void model_states_callback(gazebo_msgs::ModelStates model_states) {
            model_idx_ = getModelIndex(model_states.name, model_name_);
            if (!ready_to_pub_ || model_idx_ == -1) {
                return;
            }
            ready_to_pub_ = false;
            model_pose_ = model_states.pose[model_idx_];
            ROS_INFO_STREAM(model_pose_); //TODO: debug stream
        }

        void timerCallback(const ros::TimerEvent&) {
            ready_to_pub_ = true;
        } 

};

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "gazebo_tracker");

    Tracker tracker;

    ros::spin();
    
    return 0;
}