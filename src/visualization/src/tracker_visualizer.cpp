#include "tracker_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <tuple>

using std::placeholders::_1;

namespace visualization {
  TrackerVisualizer::TrackerVisualizer(void) : Node("tracker_visualizer") {
    // Subscriber
    tracker_subscribers_ = this->create_subscription<tracker_msgs::msg::TrackerScan>(
      "/tracker/tracker_scan", 10, std::bind(&TrackerVisualizer::TrackerMessageCallback, this, _1));

    // Publisher
    objects_marker_publishers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vis/tracker", 10);
  }

  visualization_msgs::msg::MarkerArray TrackerVisualizer::TrackerMessageCallback(const tracker_msgs::msg::TrackerScan & tracker_scan_msg) {
    visualization_msgs::msg::MarkerArray tracker_objects_markers;

    int32_t id = 0;
    std::transform(tracker_scan_msg.objects.begin(), tracker_scan_msg.objects.end(),
      std::back_inserter(tracker_objects_markers.markers),
      [&id,this](const tracker_msgs::msg::Object & object) -> visualization_msgs::msg::Marker {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "tracker_vis";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = object.position.x;
        marker.pose.position.y = object.position.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        return marker;
      }
    );

    RCLCPP_INFO(this->get_logger(), "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");

    objects_marker_publishers_->publish(tracker_objects_markers);
  }
}