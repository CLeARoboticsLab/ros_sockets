#pragma once

#include <condition_variable>
#include <mutex>
#include <vector>

struct VelocityCommand {
  	double x{0.0};
  	double phi{0.0};
};

struct ControlData {
	public:
		void update(std::vector<VelocityCommand> value);
		auto get_new_data() -> std::vector<VelocityCommand>;
		auto has_new_data() const -> bool;

	private:
		mutable std::mutex data_mutex;
		std::vector<VelocityCommand> data{};
		std::condition_variable condition_variable;
		bool is_updated{false};
};