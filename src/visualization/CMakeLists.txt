cmake_minimum_required(VERSION 3.8)
project(visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(tracker_msgs REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(radar_processor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(tracker_visualization
  src/tracker_visualization.cpp
)
ament_target_dependencies(tracker_visualization rclcpp std_msgs tracker_msgs)


add_executable(radar_preprocessor_visualization
  src/radar_preprocessor_visualization_node.cpp
  src/radar_preprocessor_visualizer.cpp
)
target_include_directories(radar_preprocessor_visualization
  PRIVATE
    include
)
ament_target_dependencies(radar_preprocessor_visualization
  rclcpp
  std_msgs
  radar_processor_msgs
  visualization_msgs
  tf2_geometry_msgs
)



add_executable(radar_visualization
  src/radar_visualization_node.cpp
  src/radar_visualizer.cpp
)
target_include_directories(radar_visualization
  PRIVATE
    include
)
ament_target_dependencies(radar_visualization
  rclcpp
  std_msgs
  radar_msgs
  visualization_msgs
)

install(TARGETS
  radar_visualization
  tracker_visualization
  radar_preprocessor_visualization
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
