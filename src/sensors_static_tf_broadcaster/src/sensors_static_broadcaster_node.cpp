#include <map>
#include <memory>
#include <numbers>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

namespace tf_broadcaster {
  constexpr auto rad2deg = std::numbers::pi_v<float> / 180.0;

  class SensorsStaticTfBroadcaster : public rclcpp::Node {
    public:
      explicit SensorsStaticTfBroadcaster(void) : Node("sensors_static_tf_broadcaster") {
        // Timer
        timer_ = this->create_wall_timer(100ms, std::bind(&SensorsStaticTfBroadcaster::SentTransforms, this));

        // Declare parameters
        declare_parameter("radars.radar_front.id", 1);
        declare_parameter("radars.radar_front.x", 3.7f);
        declare_parameter("radars.radar_front.y", 0.0f);
        declare_parameter("radars.radar_front.z", 0.2f);
        declare_parameter("radars.radar_front.roll", 0.0f);
        declare_parameter("radars.radar_front.pitch", 0.0f);
        declare_parameter("radars.radar_front.yaw", 0.0f * rad2deg);

        declare_parameter("radars.radar_rear.id", 6);
        declare_parameter("radars.radar_rear.x", -1.0f);
        declare_parameter("radars.radar_rear.y", 0.0f);
        declare_parameter("radars.radar_rear.z", 0.2f);
        declare_parameter("radars.radar_rear.roll", 0.0f);
        declare_parameter("radars.radar_rear.pitch", 0.0f);
        declare_parameter("radars.radar_rear.yaw", -180.0f * rad2deg);

        declare_parameter("radars.radar_fr.id", 2);
        declare_parameter("radars.radar_fr.x", 2.8f);
        declare_parameter("radars.radar_fr.y", -0.9f);
        declare_parameter("radars.radar_fr.z", 0.2f);
        declare_parameter("radars.radar_fr.roll", 0.0f);
        declare_parameter("radars.radar_fr.pitch", 0.0f);
        declare_parameter("radars.radar_fr.yaw", -45.0f * rad2deg);

        declare_parameter("radars.radar_rl.id", 3);
        declare_parameter("radars.radar_rl.x", 0.0f);
        declare_parameter("radars.radar_rl.y", 0.0f);
        declare_parameter("radars.radar_rl.z", 0.2f);
        declare_parameter("radars.radar_rl.roll", 0.0f);
        declare_parameter("radars.radar_rl.pitch", 0.0f);
        declare_parameter("radars.radar_rl.yaw", 135.0f * rad2deg);

        declare_parameter("radars.radar_fl.id", 4);
        declare_parameter("radars.radar_fl.x", 2.8f);
        declare_parameter("radars.radar_fl.y", 0.9f);
        declare_parameter("radars.radar_fl.z", 0.2f);
        declare_parameter("radars.radar_fl.roll", 0.0f);
        declare_parameter("radars.radar_fl.pitch", 0.0f);
        declare_parameter("radars.radar_fl.yaw", 45.0f * rad2deg);
        
        declare_parameter("radars.radar_rr.id", 5);
        declare_parameter("radars.radar_rr.x", 0.0f);
        declare_parameter("radars.radar_rr.y", -0.9f);
        declare_parameter("radars.radar_rr.z", 0.2f);
        declare_parameter("radars.radar_rr.roll", 0.0f);
        declare_parameter("radars.radar_rr.pitch", 0.0f);
        declare_parameter("radars.radar_rr.yaw", -135.0f * rad2deg);

        // Make broadcasters
        tf_static_broadcasters_["radar_front"] = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_static_broadcasters_["radar_rear"] = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_static_broadcasters_["radar_fl"] = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_static_broadcasters_["radar_fr"] = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_static_broadcasters_["radar_rl"] = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_static_broadcasters_["radar_rr"] = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Publish static transforms once at startup
        this->MakeTransforms("radar_front");
        this->MakeTransforms("radar_rear");
        this->MakeTransforms("radar_fl");
        this->MakeTransforms("radar_fr");
        this->MakeTransforms("radar_rl");
        this->MakeTransforms("radar_rr");
      }

    private:
      void MakeTransforms(const std::string radar_name) {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "/" + radar_name;

        get_parameter("radars." + radar_name + ".x", t.transform.translation.x);
        get_parameter("radars." + radar_name + ".y", t.transform.translation.y);
        get_parameter("radars." + radar_name + ".z", t.transform.translation.z);

        float roll, pitch, yaw;
        get_parameter("radars." + radar_name + ".x", roll);
        get_parameter("radars." + radar_name + ".y", pitch);
        get_parameter("radars." + radar_name + ".z", yaw);

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_static_broadcasters_[radar_name]->sendTransform(t);
      }

      void SentTransforms(void) {
        this->MakeTransforms("radar_front");
        this->MakeTransforms("radar_rear");
        this->MakeTransforms("radar_fl");
        this->MakeTransforms("radar_fr");
        this->MakeTransforms("radar_rl");
        this->MakeTransforms("radar_rr");
      }

      std::map<std::string, std::shared_ptr<tf2_ros::StaticTransformBroadcaster>> tf_static_broadcasters_;
      rclcpp::TimerBase::SharedPtr timer_;
  };
} //  tf_broadcaster

int main(int argc, char * argv[]) {
  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tf_broadcaster::SensorsStaticTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}