#ifndef RADAR_PREPROCESSOR_INCLUDE_RADAR_PREPROCESSOR_WRAPPER_HPP_
#define RADAR_PREPROCESSOR_INCLUDE_RADAR_PREPROCESSOR_WRAPPER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "radar_processor.hpp"

#include "radar_msgs/msg/radar_scan.hpp"

using std::placeholders::_1;

namespace radar_preprocessor {
  class RadarPreprocessorWrapper : public rclcpp::Node {
    public:
      RadarPreprocessorWrapper(void);

    private:
      void RadarMessageCallback(const radar_msgs::msg::RadarScan & radar_scan_msg);

      rclcpp::Subscription<radar_msgs::msg::RadarScan>::SharedPtr radar_subscriber_;
      std::unique_ptr<measurements::radar::RadarProcessor> radar_processor_;
  };
} //  namespace radar_preprocessor

#endif  //  RADAR_PREPROCESSOR_INCLUDE_RADAR_PREPROCESSOR_WRAPPER_HPP_
