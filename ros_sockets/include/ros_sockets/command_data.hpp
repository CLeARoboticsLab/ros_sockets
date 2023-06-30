#pragma once

#include <condition_variable>
#include <mutex>

class CommandData
{
  public:
    void update();
    auto getNewData() -> bool;

  private:
    mutable std::mutex data_mutex_;
    std::condition_variable condition_variable_;
    bool is_updated_{false};
};