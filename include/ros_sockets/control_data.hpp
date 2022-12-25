#pragma once

#include <condition_variable>
#include <mutex>
#include <vector>

struct VelocityCommand
{
  double x{0.0};
  double phi{0.0};
};

class ControlData
{
  public:
    void update(std::vector<VelocityCommand> value);
    auto getNewData() -> std::vector<VelocityCommand>;
    auto hasNewData() const -> bool;

  private:
    mutable std::mutex data_mutex_;
    std::vector<VelocityCommand> data_{};
    std::condition_variable condition_variable_;
    bool is_updated_{false};
};
