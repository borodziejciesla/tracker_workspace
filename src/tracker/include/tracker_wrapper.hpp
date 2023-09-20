#ifndef TRACKER_INCLUDE_TRACKER_WRAPPER_HPP_
#define TRACKER_INCLUDE_TRACKER_WRAPPER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "tracker_msgs/msg/tracker_scan.hpp"
#include "radar_msgs/msg/radar_scan.hpp"

namespace tracker_wrapper {
  class TrackerWrapper : public rclcpp::Node {
    public:
      TrackerWrapper(void);

    private:
      rclcpp::Publisher<tracker_msgs::msg::TrackerScan>::SharedPtr tracker_spublisher_;
      tracker_msgs::msg::TrackerScan tracker_scan_;
  };
} // namespace tracker_wrapper

#endif  //  TRACKER_INCLUDE_TRACKER_WRAPPER_HPP_
