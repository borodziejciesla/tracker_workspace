#include <tracker_visualizer.hpp>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<visualization::TrackerVisualizer>());
  rclcpp::shutdown();
  return 0;
}
