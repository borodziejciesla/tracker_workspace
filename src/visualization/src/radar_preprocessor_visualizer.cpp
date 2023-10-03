#include "radar_preprocessor_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <tuple>

using std::placeholders::_1;

namespace visualization {
  RadarPreprocessorVisualizer::RadarPreprocessorVisualizer(void) : Node("radar_visualizer") {
    // Subscriber
    radar_processor_subscribers_ = this->create_subscription<radar_processor_msgs::msg::ScanObjects>(
      "/radar/objects", 10, std::bind(&RadarPreprocessorVisualizer::RadarPreprocessorMessageCallback, this, _1));
    // Publisher
    objects_marker_publishers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vis/radar_objects", 10);
  }

  void RadarPreprocessorVisualizer::RadarPreprocessorMessageCallback(const radar_processor_msgs::msg::ScanObjects & radar_processor_scan_msg) {
    std::ignore = radar_processor_scan_msg;
  }
} //  namespace visualization
