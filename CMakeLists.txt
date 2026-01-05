cmake_minimum_required(VERSION 3.8)
project(stepper_jtc_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Build the hardware interface library
add_library(stepper_hardware_interface SHARED
  src/stepper_hardware_interface.cpp
)

target_include_directories(stepper_hardware_interface PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(stepper_hardware_interface
  PRIVATE "STEPPER_CONTROL_BUILDING_DLL"
)

ament_target_dependencies(stepper_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface stepper_hardware_interface.xml)

# Build the action client library
add_library(stepper_trajectory_client SHARED
  src/stepper_trajectory_client.cpp
)

target_include_directories(stepper_trajectory_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(stepper_trajectory_client
  PRIVATE "STEPPER_CONTROL_BUILDING_DLL"
)

ament_target_dependencies(stepper_trajectory_client
  rclcpp
  rclcpp_action
  rclcpp_components
  control_msgs
  trajectory_msgs
)

# Register as a composable node and create executable
rclcpp_components_register_node(stepper_trajectory_client 
  PLUGIN "stepper_jtc_control::StepperTrajectoryClient" 
  EXECUTABLE stepper_trajectory_client_node
)

# Build the production multi-joint client
add_library(multi_joint_trajectory_client SHARED
  src/multi_joint_trajectory_client.cpp
)

target_include_directories(multi_joint_trajectory_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(multi_joint_trajectory_client
  PRIVATE "STEPPER_CONTROL_BUILDING_DLL"
)

ament_target_dependencies(multi_joint_trajectory_client
  rclcpp
  rclcpp_action
  rclcpp_components
  control_msgs
  sensor_msgs
  trajectory_msgs
)

rclcpp_components_register_node(multi_joint_trajectory_client
  PLUGIN "stepper_jtc_control::MultiJointTrajectoryClient"
  EXECUTABLE multi_joint_trajectory_client_node
)

# Build the trajectory logger
add_executable(trajectory_logger
  src/trajectory_logger.cpp
)

ament_target_dependencies(trajectory_logger
  rclcpp
  control_msgs
)

# Install
install(TARGETS
  stepper_hardware_interface
  stepper_trajectory_client
  multi_joint_trajectory_client 
 ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  trajectory_logger
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(stepper_hardware_interface stepper_trajectory_client multi_joint_trajectory_client)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
