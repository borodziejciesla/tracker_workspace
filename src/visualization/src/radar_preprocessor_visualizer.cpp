#include "radar_preprocessor_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <tuple>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

namespace visualization {
  RadarPreprocessorVisualizer::RadarPreprocessorVisualizer(void) : Node("radar_preprocessor_visualizer") {
    // Subscriber
    radar_processor_subscribers_ = this->create_subscription<radar_processor_msgs::msg::ScanObjects>(
      "/radar/objects", 10, std::bind(&RadarPreprocessorVisualizer::RadarPreprocessorMessageCallback, this, _1));
    // Publisher
    objects_marker_publishers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vis/radar_objects", 10);
  }

  void RadarPreprocessorVisualizer::RadarPreprocessorMessageCallback(const radar_processor_msgs::msg::ScanObjects & radar_processor_scan_msg) {
    objects_array_.markers.clear();
    
    std::transform(radar_processor_scan_msg.objects.begin(), radar_processor_scan_msg.objects.end(),
      std::back_inserter(objects_array_.markers),
      [this](const radar_processor_msgs::msg::MovingObject & radar_object) {
        return ConvertObjectToMarker(radar_object, 0);
      }
    );

    objects_marker_publishers_->publish(objects_array_);
  }

  visualization_msgs::msg::Marker RadarPreprocessorVisualizer::ConvertObjectToMarker(const radar_processor_msgs::msg::MovingObject & radar_object, const int32_t id) {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "frame_id";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "radar_object_vis";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = radar_object.pose.x;
    marker.pose.position.y = radar_object.pose.y;
    marker.pose.position.z = 0.0;
    tf2::Quaternion my_quaternion;
    my_quaternion.setRPY(0.0, 0.0, radar_object.pose.orientation);
    marker.pose.orientation.x = my_quaternion.getX();
    marker.pose.orientation.y = my_quaternion.getY();
    marker.pose.orientation.z = my_quaternion.getZ();
    marker.pose.orientation.w = my_quaternion.getW();
    marker.scale.x = radar_object.size.length;
    marker.scale.y = radar_object.size.width;
    marker.scale.z = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
  }
} //  namespace visualization
