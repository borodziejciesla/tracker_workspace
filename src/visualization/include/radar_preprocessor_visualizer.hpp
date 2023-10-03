#ifndef VISUALIZATION_INCLUDE_RADAR_PREPROCESSOR_VISUALIZER_HPP_
#define VISUALIZATION_INCLUDE_RADAR_PREPROCESSOR_VISUALIZER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "radar_processor_msgs/msg/scan_objects.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

namespace visualization {
  class RadarPreprocessorVisualizer : public rclcpp::Node {
    public:
      RadarPreprocessorVisualizer(void);

    private:
      // visualization_msgs::msg::MarkerArray RadarPreprocessorMessageCallback(const radar_msgs::msg::RadarScan & radar_scan_msg, const std::string & frame_id);
      void RadarPreprocessorMessageCallback(const radar_processor_msgs::msg::ScanObjects & radar_processor_scan_msg);
      // void RadarMessageCallbackRear(const radar_msgs::msg::RadarScan & radar_scan_msg);
      // void RadarMessageCallbackFrontLeft(const radar_msgs::msg::RadarScan & radar_scan_msg);
      // void RadarMessageCallbackFrontRight(const radar_msgs::msg::RadarScan & radar_scan_msg);
      // void RadarMessageCallbackRearLeft(const radar_msgs::msg::RadarScan & radar_scan_msg);
      // void RadarMessageCallbackRearRight(const radar_msgs::msg::RadarScan & radar_scan_msg);

      rclcpp::Subscription<radar_processor_msgs::msg::ScanObjects>::SharedPtr radar_processor_subscribers_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr objects_marker_publishers_;
  };
} //  namespace visualization

#endif  //  VISUALIZATION_INCLUDE_RADAR_PREPROCESSOR_VISUALIZER_HPP_
