#include "ros_sockets/control_data.hpp"

#include <chrono>

#include <ros/ros.h>

using namespace std::chrono_literals;

void ControlData::update(std::vector<VelocityCommand> value)
{
	std::unique_lock<std::mutex> lock(data_mutex_);
	data_ = std::move(value);
	is_updated_ = true;
	condition_variable_.notify_all();
}

auto ControlData::hasNewData() const -> bool
{
	std::unique_lock<std::mutex> lock(data_mutex_);
	return is_updated_;
}

auto ControlData::getNewData() -> std::vector<VelocityCommand>
{
	std::unique_lock<std::mutex> lock(data_mutex_);
	condition_variable_.wait_for(lock, 2s, [this] { return is_updated_; });
  if (is_updated_)
  {
    is_updated_ = false;
    return data_;
  }
  else
  {
    std::vector<VelocityCommand> empty_vector;
    return empty_vector;
  }
}