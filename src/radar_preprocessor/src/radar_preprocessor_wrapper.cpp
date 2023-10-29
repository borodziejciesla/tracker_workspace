#include "radar_preprocessor_wrapper.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <tuple>

#include "processor_calibration.hpp"

using namespace std::chrono_literals;

namespace radar_preprocessor {
  RadarPreprocessorWrapper::RadarPreprocessorWrapper(void) : Node("radar_preprocessor") {
    // Create processor
    measurements::radar::ProcessorCalibration radar_processor_calibrations;

    radar_processor_calibrations.dealiaser_calibration.dealiaser_threshold = 1.0f;

    radar_processor_calibrations.segmentator_calibration.probability_hreshold = 0.85f;
    radar_processor_calibrations.segmentator_calibration.minimum_detection_in_segment = 2u;

    radar_processor_calibrations.velocity_estimator_calibration.maximum_iterations_number = 20u;
    radar_processor_calibrations.velocity_estimator_calibration.inlier_threshold = 0.25f;

    radar_processor_ = std::make_unique<measurements::radar::RadarProcessor>(radar_processor_calibrations);

    // Prepare subscriber
    radar_subscriber_ = this->create_subscription<radar_msgs::msg::RadarScan>(
      "/radar_scan_front", 10, std::bind(&RadarPreprocessorWrapper::RadarMessageCallback, this, _1));

    // Prepare publisher
    objects_publisher_ = this->create_publisher<radar_processor_msgs::msg::ScanObjects>("/radar/objects", 10);
  }

  void RadarPreprocessorWrapper::RadarMessageCallback(const radar_msgs::msg::RadarScan & radar_scan_msg) {
    measurements::radar::RadarScan radar_scan;

    radar_scan.sensor_origin.x = radar_scan_msg.sensor_origin.x;
    radar_scan.sensor_origin.y = radar_scan_msg.sensor_origin.y;
    radar_scan.sensor_origin.z = radar_scan_msg.sensor_origin.z;
    radar_scan.sensor_origin.roll = radar_scan_msg.sensor_origin.roll;
    radar_scan.sensor_origin.pitch = radar_scan_msg.sensor_origin.pitch;
    radar_scan.sensor_origin.yaw = radar_scan_msg.sensor_origin.yaw;

    radar_scan.aliasing_period = 0.0f;

    size_t id = 0u;
    std::transform(radar_scan_msg.detections.begin(), radar_scan_msg.detections.end(),
      std::back_inserter(radar_scan.detections),
      [&id] (const auto & detection_msg) -> measurements::radar::RadarDetection {
        measurements::radar::RadarDetection detection;

        detection.id = ++id;

        detection.range = detection_msg.range;
        detection.range_std = detection_msg.range_std;
        detection.azimuth = detection_msg.azimuth;
        detection.azimuth_std = detection_msg.azimuth_std;
        detection.elevaion = 0.0f;
        detection.elevaion = 0.01f;
        detection.range_rate = detection_msg.range_rate;
        detection.range_rate_std = detection_msg.range_rate_std;

        detection.range_rate_compensated = detection_msg.range_rate;
        detection.range_rate_compensated_std = detection_msg.range_rate_std;

        detection.x = detection_msg.range * std::cos(detection_msg.azimuth);
        detection.y = detection_msg.range * std::sin(detection_msg.azimuth);
        detection.z = 0.0f;

        const auto c_sqr = std::pow(std::cos(detection_msg.azimuth), 2u);
        const auto s_sqr = std::pow(std::sin(detection_msg.azimuth), 2u);
        const auto r_sqr = std::pow(detection_msg.range, 2u);
        const auto r_cov = std::pow(detection.range_std, 2u);
        const auto az_cov = std::pow(detection.azimuth_std, 2u);

        detection.x_std = std::sqrt(c_sqr * r_cov + r_sqr * s_sqr * az_cov);
        detection.y_std = std::sqrt(s_sqr * r_cov + r_sqr * c_sqr * az_cov);

        detection.dealiasing_status = measurements::radar::DealiasingStatus::StaticVelocityProfileDealiased;
        detection.moving_status = measurements::radar::MovingStatus::Static;

        return detection;
      }
    );

    const auto output = radar_processor_->ProcessScan(radar_scan);

    if (output.has_value()) {
      const auto objects = std::get<0>(output.value());

      scan_objects_.radar_id = 0u;

      // Copy detections
      scan_objects_.detections.clear();
      std::copy(radar_scan_msg.detections.begin(), radar_scan_msg.detections.end(), std::back_inserter(scan_objects_.detections));

      // Convert objects
      scan_objects_.objects.clear();
      std::transform(objects.begin(), objects.end(),
        std::back_inserter(scan_objects_.objects),
        [radar_scan] (const auto & object) {
          radar_processor_msgs::msg::MovingObject object_msg;

          const auto cos_yaw = std::cos(-radar_scan.sensor_origin.yaw);
          const auto sin_yaw = std::sin(-radar_scan.sensor_origin.yaw);

          const auto rotated_x = object.object_center.x * cos_yaw - object.object_center.y * sin_yaw;
          const auto rotated_y = object.object_center.x * sin_yaw + object.object_center.y * cos_yaw;

          object_msg.pose.x = rotated_x + radar_scan.sensor_origin.x;
          object_msg.pose.y = rotated_y + radar_scan.sensor_origin.y;
          object_msg.pose.orientation = object.object_center.orientation + radar_scan.sensor_origin.yaw;

          object_msg.velocity.vx = object.object_velocity.vx;
          object_msg.velocity.vy = object.object_velocity.vy;
          
          object_msg.size.length = object.object_size.length;
          object_msg.size.width = object.object_size.width;

          return object_msg;
        }
      );

      objects_publisher_->publish(scan_objects_);
    }
  }
} //  radar_preprocessor
