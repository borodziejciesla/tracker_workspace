#ifndef RADAR_PREPROCESSOR_INCLUDE_RADAR_PREPROCESSOR_WRAPPER_HPP_
#define RADAR_PREPROCESSOR_INCLUDE_RADAR_PREPROCESSOR_WRAPPER_HPP_

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "radar_processor.hpp"

#include "radar_msgs/msg/radar_scan.hpp"
#include "radar_processor_msgs/msg/scan_objects.hpp"

using std::placeholders::_1;

namespace radar_preprocessor {
  class RadarPreprocessorWrapper : public rclcpp::Node {
    public:
      RadarPreprocessorWrapper(void);

    private:
      void RadarMessageCallback(const radar_msgs::msg::RadarScan & radar_scan_msg, const std::string & radar_name);
      void RadarMessageCallbackFront(const radar_msgs::msg::RadarScan & radar_scan_msg);
      void RadarMessageCallbackFrontLeft(const radar_msgs::msg::RadarScan & radar_scan_msg);
      void RadarMessageCallbackFrontRight(const radar_msgs::msg::RadarScan & radar_scan_msg);
      void RadarMessageCallbackRearLeft(const radar_msgs::msg::RadarScan & radar_scan_msg);
      void RadarMessageCallbackRearRight(const radar_msgs::msg::RadarScan & radar_scan_msg);
      void RadarMessageCallbackRear(const radar_msgs::msg::RadarScan & radar_scan_msg);
      std::map<std::string, rclcpp::Subscription<radar_msgs::msg::RadarScan>::SharedPtr> radar_subscribers_;
      std::map<std::string, rclcpp::Publisher<radar_processor_msgs::msg::ScanObjects>::SharedPtr> objects_publishers_;

      std::unique_ptr<measurements::radar::RadarProcessor> radar_processor_;

      radar_processor_msgs::msg::ScanObjects scan_objects_;
  };
} //  namespace radar_preprocessor

#endif  //  RADAR_PREPROCESSOR_INCLUDE_RADAR_PREPROCESSOR_WRAPPER_HPP_
