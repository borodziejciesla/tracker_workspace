#include "tracker_wrapper.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <tuple>

using std::placeholders::_1;

using namespace std::chrono_literals;

namespace tracker_wrapper {
  TrackerWrapper::TrackerWrapper(void) : Node("tracker_wrapper") {
    // Initialize subscriber
    radar_subscriber_ = this->create_subscription<radar_processor_msgs::msg::ScanObjects>(
      "/radar/objects", 10, std::bind(&TrackerWrapper::RadarPreprocessorSubscriberCallback, this, _1));

    // Initialize publisher
    tracker_publisher_ = this->create_publisher<tracker_msgs::msg::TrackerScan>("tracker_scan", 10);

    // Initialize tracker
    calibrations_.observation_matrix = Eigen::Matrix<double, 2u, 4u>::Zero();
    calibrations_.observation_matrix(0u, 0u) = 1.0;
    calibrations_.observation_matrix(1u, 1u) = 1.0;

    calibrations_.measurement_covariance = 0.2 * Eigen::Matrix<double, 2u, 2u>::Identity();

    gm_phd_tracker_ = std::make_unique<mot::GmPhdCvPose>(calibrations_);
  }

  void TrackerWrapper::RadarPreprocessorSubscriberCallback(const radar_processor_msgs::msg::ScanObjects & radar_preprocessor_msg) {
    // Convert input
    const auto measurements = ConvertPreprocessorScanToPhdInput(radar_preprocessor_msg);
    // Run filter
    gm_phd_tracker_->Run(0.0, measurements);
    const auto objects = gm_phd_tracker_->GetObjects();

    // Publish
    tracker_scan_ = ConvertPhdOutputToMessage(objects);
    tracker_publisher_->publish(tracker_scan_);
  }

  std::vector<mot::GmPhdCvPose::Measurement> TrackerWrapper::ConvertPreprocessorScanToPhdInput(const radar_processor_msgs::msg::ScanObjects & radar_preprocessor_msg) const {
    std::vector<mot::GmPhdCvPose::Measurement> measurements;
    std::transform(radar_preprocessor_msg.objects.begin(), radar_preprocessor_msg.objects.end(),
      std::back_inserter(measurements),
      [this](const radar_processor_msgs::msg::MovingObject & preprocessor_object) {
        mot::GmPhdCvPose::Measurement measurement;

        measurement.value(0) = preprocessor_object.pose.x;
        measurement.value(1) = preprocessor_object.pose.y;
        
        measurement.covariance(0, 0) = preprocessor_object.pose.cov_x;
        measurement.covariance(0, 1) = preprocessor_object.pose.cov_xy;
        measurement.covariance(1, 0) = preprocessor_object.pose.cov_xy;
        measurement.covariance(1, 1) = preprocessor_object.pose.cov_y;

        return measurement;
      }
    );
    return measurements;
  }

  tracker_msgs::msg::TrackerScan TrackerWrapper::ConvertPhdOutputToMessage(const std::vector<mot::GmPhdCvPose::Object> & objects) const {
    tracker_msgs::msg::TrackerScan tracker_scan;

    tracker_scan.timestamp = 0.0;

    tracker_scan.objects_number = objects.size();

    std::transform(objects.begin(), objects.end(),
      std::back_inserter(tracker_scan.objects),
      [](const mot::GmPhdCvPose::Object & object) {
        tracker_msgs::msg::Object object_msg;

        object_msg.position.x = object.value(0u);
        object_msg.position.y = object.value(1u);

        return object_msg;
      }
    );

    return tracker_scan;
  }
} //  tracker_wrapper
