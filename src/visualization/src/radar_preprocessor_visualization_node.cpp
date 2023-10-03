#include "radar_preprocessor_visualizer.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<visualization::RadarPreprocessorVisualizer>());
  rclcpp::shutdown();
  return 0;
}