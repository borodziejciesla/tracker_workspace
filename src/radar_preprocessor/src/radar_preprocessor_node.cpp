#include "radar_preprocessor_wrapper.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<radar_preprocessor::RadarPreprocessorWrapper>());
  rclcpp::shutdown();
  return 0;
}
