#include "radar_preprocessor_wrapper.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <tuple>

#include "processor_calibration.hpp"

using namespace std::chrono_literals;

namespace radar_preprocessor {
  measurements::radar::ProcessorCalibration radar_processor_calibrations;

  RadarPreprocessorWrapper::RadarPreprocessorWrapper()
    : Node("radar_preprocessor")
    , radar_processor_{std::make_unique<measurements::radar::RadarProcessor>(radar_processor_calibrations)} {
    radar_subscriber_ = this->create_subscription<radar_msgs::msg::RadarScan>(
      "/radar_scan_front", 10, std::bind(&RadarPreprocessorWrapper::RadarMessageCallback, this, _1));
  }

  void RadarPreprocessorWrapper::RadarMessageCallback(const radar_msgs::msg::RadarScan & radar_scan_msg) {
    std::ignore = radar_scan_msg;
    RCLCPP_INFO(this->get_logger(), "Radar Preprocessor");
  }
} //  radar_preprocessor
