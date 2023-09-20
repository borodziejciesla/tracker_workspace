#include "tracker_wrapper.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace tracker_wrapper {
  TrackerWrapper::TrackerWrapper(void) : Node("tracker_wrapper") {
    tracker_spublisher_ = this->create_publisher<tracker_msgs::msg::TrackerScan>("tracker_scan", 10);
  }
} //  tracker_wrapper
