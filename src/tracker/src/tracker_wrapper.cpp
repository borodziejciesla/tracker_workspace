#include "tracker_wrapper.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <tuple>

using std::placeholders::_1;

using namespace std::chrono_literals;

namespace tracker_wrapper {
  TrackerWrapper::TrackerWrapper(void) : Node("tracker_wrapper") {
    // Initialize subscriber
    radar_subscriber_ = this->create_subscription<radar_msgs::msg::RadarScan>(
      "/radar_preprocessor/radar_scan", 10, std::bind(&TrackerWrapper::RadarSubscriberCallback, this, _1));

    // Initialize publisher
    tracker_publisher_ = this->create_publisher<tracker_msgs::msg::TrackerScan>("tracker_scan", 10);
  }

  void TrackerWrapper::RadarSubscriberCallback(const radar_msgs::msg::RadarScan & radar_msg) {
    std::ignore = radar_msg;
  }
} //  tracker_wrapper
