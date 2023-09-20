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
      void RadarSubscriberCallback(const radar_msgs::msg::RadarScan & radar_msg);

      rclcpp::Subscription<radar_msgs::msg::RadarScan>::SharedPtr radar_subscriber_;

      rclcpp::Publisher<tracker_msgs::msg::TrackerScan>::SharedPtr tracker_publisher_;
      tracker_msgs::msg::TrackerScan tracker_scan_;
  };
} // namespace tracker_wrapper

#endif  //  TRACKER_INCLUDE_TRACKER_WRAPPER_HPP_
