#ifndef VISUALIZATION_INCLUDE_RADAR_VISUALIZER_HPP_
#define VISUALIZATION_INCLUDE_RADAR_VISUALIZER_HPP_

#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "radar_msgs/msg/radar_scan.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

namespace visualization {
  class RadarVisualizer : public rclcpp::Node {
    public:
      RadarVisualizer(void);

    private:
      visualization_msgs::msg::MarkerArray RadarMessageCallback(const radar_msgs::msg::RadarScan & radar_scan_msg, const std::string & frame_id);
      void RadarMessageCallbackFront(const radar_msgs::msg::RadarScan & radar_scan_msg);
      void RadarMessageCallbackRear(const radar_msgs::msg::RadarScan & radar_scan_msg);
      void RadarMessageCallbackFrontLeft(const radar_msgs::msg::RadarScan & radar_scan_msg);
      void RadarMessageCallbackFrontRight(const radar_msgs::msg::RadarScan & radar_scan_msg);
      void RadarMessageCallbackRearLeft(const radar_msgs::msg::RadarScan & radar_scan_msg);
      void RadarMessageCallbackRearRight(const radar_msgs::msg::RadarScan & radar_scan_msg);

      std::map<std::string, rclcpp::Subscription<radar_msgs::msg::RadarScan>::SharedPtr> radar_subscribers_;
      std::map<std::string, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> detections_marker_publishers_;
  };
} //  namespace visualization

#endif  //  VISUALIZATION_INCLUDE_RADAR_VISUALIZER_HPP_
