#include "ros_sockets/command_data.hpp"

#include <chrono>

using namespace std::chrono_literals;

void CommandData::update()
{
	std::unique_lock<std::mutex> lock(data_mutex_);
	is_updated_ = true;
	condition_variable_.notify_all();
}

auto CommandData::getNewData() -> bool
{
	std::unique_lock<std::mutex> lock(data_mutex_);
	condition_variable_.wait_for(lock, 2s, [this] { return is_updated_; });
  if (is_updated_)
  {
    is_updated_ = false;
    return true;
  }
  else
  {
    return false;
  }
}