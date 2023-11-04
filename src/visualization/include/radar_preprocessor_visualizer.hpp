#ifndef VISUALIZATION_INCLUDE_RADAR_PREPROCESSOR_VISUALIZER_HPP_
#define VISUALIZATION_INCLUDE_RADAR_PREPROCESSOR_VISUALIZER_HPP_

#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "radar_processor_msgs/msg/scan_objects.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

namespace visualization {
  class RadarPreprocessorVisualizer : public rclcpp::Node {
    public:
      RadarPreprocessorVisualizer(void);

    private:
      void RadarPreprocessorMessageCallback(const radar_processor_msgs::msg::ScanObjects & radar_processor_scan_msg, const std::string & radar_name);
      void RadarPreprocessorMessageCallbackFront(const radar_processor_msgs::msg::ScanObjects & radar_processor_scan_msg);
      void RadarPreprocessorMessageCallbackFrontLeft(const radar_processor_msgs::msg::ScanObjects & radar_processor_scan_msg);
      void RadarPreprocessorMessageCallbackFrontRight(const radar_processor_msgs::msg::ScanObjects & radar_processor_scan_msg);
      void RadarPreprocessorMessageCallbackRearLeft(const radar_processor_msgs::msg::ScanObjects & radar_processor_scan_msg);
      void RadarPreprocessorMessageCallbackRearRight(const radar_processor_msgs::msg::ScanObjects & radar_processor_scan_msg);
      void RadarPreprocessorMessageCallbackRear(const radar_processor_msgs::msg::ScanObjects & radar_processor_scan_msg);
      visualization_msgs::msg::Marker ConvertObjectToMarker(const radar_processor_msgs::msg::MovingObject & radar_object, const int32_t id);

      std::map<std::string, rclcpp::Subscription<radar_processor_msgs::msg::ScanObjects>::SharedPtr> radar_processor_subscribers_;
      std::map<std::string, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> objects_marker_publishers_;
      visualization_msgs::msg::MarkerArray objects_array_;
  };
} //  namespace visualization

#endif  //  VISUALIZATION_INCLUDE_RADAR_PREPROCESSOR_VISUALIZER_HPP_
