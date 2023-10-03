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

  // visualization_msgs::msg::MarkerArray RadarPreprocessorVisualizer::RadarMessageCallback(const radar_msgs::msg::RadarScan & radar_scan_msg, const std::string & frame_id) {
  //   visualization_msgs::msg::MarkerArray vis_detections;
  //   int id = 0;

  //   std::transform(radar_scan_msg.detections.begin(), radar_scan_msg.detections.end(),
  //     std::back_inserter(vis_detections.markers),
  //     [this, frame_id, &id](const radar_msgs::msg::Detection & detection) {
  //       visualization_msgs::msg::Marker marker;

  //       marker.header.frame_id = frame_id;
  //       marker.header.stamp = this->get_clock()->now();
  //       marker.ns = "radar_vis";
  //       marker.id = id++;
  //       marker.type = visualization_msgs::msg::Marker::SPHERE;
  //       marker.action = visualization_msgs::msg::Marker::ADD;
  //       marker.pose.position.x = detection.range * std::cos(detection.azimuth);
  //       marker.pose.position.y = detection.range * std::sin(detection.azimuth);
  //       marker.pose.position.z = 0.0;
  //       marker.pose.orientation.x = 0.0;
  //       marker.pose.orientation.y = 0.0;
  //       marker.pose.orientation.z = 0.0;
  //       marker.pose.orientation.w = 1.0;
  //       marker.scale.x = 1.1;
  //       marker.scale.y = 1.1;
  //       marker.scale.z = 1.1;
  //       marker.color.a = 1.0; // Don't forget to set the alpha!
  //       marker.color.r = 0.0;
  //       marker.color.g = 0.0;
  //       marker.color.b = 1.0;

  //       return marker;
  //     }
  //   );

  //   return vis_detections;
  // }

  // void RadarVisualizer::RadarMessageCallbackFront(const radar_msgs::msg::RadarScan & radar_scan_msg) {
  //   const auto marker_array = RadarMessageCallback(radar_scan_msg, "/radar_front");
  //   detections_marker_publishers_["/radar_scan_front"]->publish(marker_array);
  // }

  // void RadarVisualizer::RadarMessageCallbackRear(const radar_msgs::msg::RadarScan & radar_scan_msg) {
  //   const auto marker_array = RadarMessageCallback(radar_scan_msg, "/radar_rear");
  //   detections_marker_publishers_["/radar_scan_rear"]->publish(marker_array);
  // }

  // void RadarVisualizer::RadarMessageCallbackFrontLeft(const radar_msgs::msg::RadarScan & radar_scan_msg) {
  //   const auto marker_array = RadarMessageCallback(radar_scan_msg, "/radar_fl");
  //   detections_marker_publishers_["/radar_scan_fl"]->publish(marker_array);
  // }

  // void RadarVisualizer::RadarMessageCallbackFrontRight(const radar_msgs::msg::RadarScan & radar_scan_msg) {
  //   const auto marker_array = RadarMessageCallback(radar_scan_msg, "/radar_fr");
  //   detections_marker_publishers_["/radar_scan_fr"]->publish(marker_array);
  // }

  // void RadarVisualizer::RadarMessageCallbackRearLeft(const radar_msgs::msg::RadarScan & radar_scan_msg) {
  //   const auto marker_array = RadarMessageCallback(radar_scan_msg, "/radar_rl");
  //   detections_marker_publishers_["/radar_scan_rl"]->publish(marker_array);
  // }

  // void RadarVisualizer::RadarMessageCallbackRearRight(const radar_msgs::msg::RadarScan & radar_scan_msg) {
  //   const auto marker_array = RadarMessageCallback(radar_scan_msg, "/radar_rr");
  //   detections_marker_publishers_["/radar_scan_rr"]->publish(marker_array);
  // }
} //  namespace visualization
