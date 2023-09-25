cmake_minimum_required(VERSION 3.8)
project(radar_preprocessor)

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

find_package(radar_msgs REQUIRED)
find_package(radar_processor_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_subdirectory(include/radar_perception)

add_executable(radar_preprocessor_wrapper
  src/radar_preprocessor_node.cpp
  src/radar_preprocessor_wrapper.cpp

  $<TARGET_OBJECTS:radar_perception>
  $<TARGET_OBJECTS:dealiaser>
  $<TARGET_OBJECTS:detection_classifier>
  $<TARGET_OBJECTS:segmentator>
  $<TARGET_OBJECTS:segment_processor>
  $<TARGET_OBJECTS:velocity_estimator>
)

target_include_directories(radar_preprocessor_wrapper
  PRIVATE
    include
    include/radar_perception/include
    include/radar_perception/include/radar_io/include

    $<TARGET_PROPERTY:radar_perception,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(radar_preprocessor_wrapper
  radar_perception
  dealiaser
  detection_classifier
  segmentator
  segment_processor
  velocity_estimator
)

ament_target_dependencies(radar_preprocessor_wrapper
  rclcpp
  std_msgs
  radar_msgs
  radar_processor_msgs
)

install(TARGETS
  radar_preprocessor_wrapper
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
