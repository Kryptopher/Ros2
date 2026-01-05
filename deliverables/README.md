# Multi-Joint Trajectory Client - Deliverables

This folder contains the files needed to integrate the multi-joint trajectory client into your ROS 2 system.

## Files Included

- **multi_joint_trajectory_client.cpp** - Production-ready action client for 3-joint trajectory control
- **velocity_profile_example.csv** - Example CSV velocity profile format

## Quick Start

### 1. Copy files to your package
```bash
cp multi_joint_trajectory_client.cpp /path/to/your_package/src/
```

### 2. Add to CMakeLists.txt
```cmake
# Add the multi-joint trajectory client
add_library(multi_joint_trajectory_client SHARED
  src/multi_joint_trajectory_client.cpp
)

target_include_directories(multi_joint_trajectory_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
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

install(TARGETS multi_joint_trajectory_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```

### 3. Add dependencies to package.xml
```xml
<depend>rclcpp</depend>
<depend>rclcpp_action</depend>
<depend>rclcpp_components</depend>
<depend>control_msgs</depend>
<depend>sensor_msgs</depend>
<depend>trajectory_msgs</depend>
```

### 4. Build
```bash
colcon build --packages-select your_package_name
source install/setup.bash
```

## Usage

### With CSV file
```bash
ros2 run your_package_name multi_joint_trajectory_client_node \
  --ros-args \
  -p action_server_name:=/kinematic_controller_baseline/follow_joint_trajectory \
  -p csv_file_path:=/path/to/velocity_profile.csv
```

### With default profile
```bash
ros2 run your_package_name multi_joint_trajectory_client_node \
  --ros-args \
  -p action_server_name:=/kinematic_controller_baseline/follow_joint_trajectory
```

## CSV File Format

The CSV must have 4 columns: `time,waist_vel,shoulder_vel,elbow_vel`

Example:
```csv
time,waist_vel,shoulder_vel,elbow_vel
0.0,5.0,3.0,2.0
5.0,25.0,15.0,10.0
10.0,10.0,8.0,5.0
15.0,0.0,0.0,0.0
```

- **time**: Time in seconds (monotonically increasing)
- **waist_vel**: Velocity for waist_joint (rad/s)
- **shoulder_vel**: Velocity for shoulder_joint (rad/s)  
- **elbow_vel**: Velocity for elbow_joint (rad/s)

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `action_server_name` | `/kinematic_controller_baseline/follow_joint_trajectory` | Action server name |
| `csv_file_path` | `""` | Path to CSV file (empty = use default) |
| `sampling_rate` | `0.02` | Trajectory sampling rate (seconds) |
| `max_velocity` | `100.0` | Maximum velocity limit (rad/s) |
| `max_acceleration` | `1000.0` | Maximum acceleration limit (rad/s²) |
| `wait_for_joint_states` | `true` | Wait for /joint_states before sending |
| `joint_state_topic` | `/joint_states` | Joint states topic |
| `joint_state_timeout` | `5.0` | Timeout for joint states (seconds) |

## System Requirements

### Action Server
- Name: `/kinematic_controller_baseline/follow_joint_trajectory`
- Type: `control_msgs/action/FollowJointTrajectory`
- Must accept exactly 3 joints in order: waist_joint, shoulder_joint, elbow_joint

### Joint States
- Topic: `/joint_states`
- Must publish: waist_joint, shoulder_joint, elbow_joint

## Features

- Reads initial joint positions from /joint_states for smooth trajectory starting
- Validates velocity and acceleration limits
- Supports CSV-based or hardcoded velocity profiles
- Generates trajectory with proper positions, velocities, and accelerations
- Provides real-time feedback during execution

## Troubleshooting

**Action server not available**
- Check: `ros2 action list`
- Verify action server name parameter

**Failed to receive joint states**
- Check: `ros2 topic echo /joint_states`
- Verify all 3 joints are published
- Increase `joint_state_timeout` if needed

**CSV load failed**
- Verify file path is correct
- Check CSV format matches example
- Ensure 4 columns with numeric values
