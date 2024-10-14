cmake_minimum_required(VERSION 3.8)
project(roboclaw_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
find_package(claw_api REQUIRED)

add_library(
  roboclaw_hardware
  SHARED
  src/ros2_claw.cpp
)
target_include_directories(
  roboclaw_hardware
  PUBLIC
  include
)
ament_target_dependencies(
  roboclaw_hardware
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  claw_api
)
# prevent pluginlib from using boost
target_compile_definitions(roboclaw_hardware PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface roboclaw_hardware.xml)

install(
  TARGETS
  roboclaw_hardware
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)

  find_package(ament_cmake_gmock REQUIRED)
  
  ament_add_gmock(test_ros2_claw test/test_ros2_claw.cpp)
  target_include_directories(test_ros2_claw PRIVATE include)
  ament_target_dependencies(
    test_ros2_claw
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  roboclaw_hardware
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
