#ifndef TRACKER_INCLUDE_TRACKER_WRAPPER_HPP_
#define TRACKER_INCLUDE_TRACKER_WRAPPER_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "ghm_phd_cv_pose.hpp"

#include "tracker_msgs/msg/tracker_scan.hpp"
#include "radar_processor_msgs/msg/scan_objects.hpp"

namespace tracker_wrapper {
  class TrackerWrapper : public rclcpp::Node {
    public:
      TrackerWrapper(void);

    private:
      void RadarPreprocessorSubscriberCallback(const radar_processor_msgs::msg::ScanObjects & radar_preprocessor_msg);
      std::vector<mot::GmPhdCvPose::Measurement> ConvertPreprocessorScanToPhdInput(const radar_processor_msgs::msg::ScanObjects & radar_preprocessor_msg) const;

      rclcpp::Subscription<radar_processor_msgs::msg::ScanObjects>::SharedPtr radar_subscriber_;
      rclcpp::Publisher<tracker_msgs::msg::TrackerScan>::SharedPtr tracker_publisher_;
      tracker_msgs::msg::TrackerScan tracker_scan_;

      std::unique_ptr<mot::GmPhdCvPose> gm_phd_tracker_;
      mot::GmPhdCalibrations<4u, 2u> calibrations_;
  };
} // namespace tracker_wrapper

#endif  //  TRACKER_INCLUDE_TRACKER_WRAPPER_HPP_
