#include "control_data.hpp"
#include "ros/ros.h"
#include <chrono>
using namespace std::chrono_literals;

void ControlData::update(std::vector<VelocityCommand> value) {
	std::unique_lock<std::mutex> lock(data_mutex);
	data = std::move(value);
	is_updated = true;
	condition_variable.notify_all();
	ROS_INFO_STREAM("Control commands received");
}

auto ControlData::has_new_data() const -> bool {
	std::unique_lock<std::mutex> lock(data_mutex);
	return is_updated;
}

auto ControlData::get_new_data() -> std::vector<VelocityCommand> {
	std::unique_lock<std::mutex> lock(data_mutex);
	condition_variable.wait_for(lock, 2s, [this] { return is_updated; });
  	if (is_updated) {
    	is_updated = false;
    	return data;
  	} else {
    	std::vector<VelocityCommand> empty_vector;
    	return empty_vector;
  	}
}