#ifndef VISUALIZATION_INCLUDE_TRACKER_VISUALIZER_HPP_
#define VISUALIZATION_INCLUDE_TRACKER_VISUALIZER_HPP_

#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

#include "tracker_msgs/msg/tracker_scan.hpp"

namespace visualization {
  class TrackerVisualizer : public rclcpp::Node {
    public:
      TrackerVisualizer(void);

    private:
      visualization_msgs::msg::MarkerArray TrackerMessageCallback(const tracker_msgs::msg::TrackerScan & tracker_scan_msg);

      rclcpp::Subscription<tracker_msgs::msg::TrackerScan>::SharedPtr tracker_subscribers_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr objects_marker_publishers_;
  };
} //  namespace visualization

#endif  //  VISUALIZATION_INCLUDE_TRACKER_VISUALIZER_HPP_
