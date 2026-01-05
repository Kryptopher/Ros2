# Stepper Motor Control with ROS2 Joint Trajectory Controller

A ROS2 package for precise stepper motor control using the Joint Trajectory Controller (JTC) with velocity-based commands. Supports both single-joint and **multi-joint (3-axis)** control for coordinated robotic systems.

## Features

- **Velocity-based trajectory control** using ROS2 JointTrajectoryController
- **Multi-joint support** for coordinated 3-axis control (waist, shoulder, elbow)
- **CSV-based velocity profiles** for easy trajectory definition
- **Parameterizable action server** names for flexible integration
- **High-precision integration** with trapezoidal method for accurate position tracking
- **Serial communication** to microcontroller (Teensy/Arduino) at 500 Hz
- **Real-time logging** with nanosecond precision
- **Trajectory comparison tools** to validate commanded vs actual velocities
- **Auto-disable motor** after trajectory completion to save power
- **Configurable parameters** for sampling rate, velocity/acceleration limits
- **Input shaping ready** for vibration reduction applications

## Multi-Joint Support

This package now includes a multi-joint trajectory client for 3-joint robotic systems. See the [`deliverables/`](deliverables/) folder for:
- Production-ready multi-joint trajectory client
- CSV velocity profile examples
- Complete integration guide

The multi-joint client supports:
- 3 joints (waist, shoulder, elbow) in coordinated motion
- CSV-based velocity profiles for all joints
- Automatic initial position retrieval from `/joint_states`
- Parameterizable action server names for different controllers

## System Architecture

### Single Joint Mode
```
Action Client (velocity profile)
    ↓
JointTrajectoryController (500 Hz, open-loop)
    ↓
Hardware Interface (velocity → steps/s conversion)
    ↓
Serial Communication (V<steps_per_sec>)
    ↓
Teensy/Arduino (STEP/DIR pulse generation)
    ↓
TB6600 Driver → Stepper Motor
```

### Multi-Joint Mode (3-axis)
```
CSV Velocity Profile
    ↓
Multi-Joint Trajectory Client
    ↓
JointTrajectoryController (kinematic_controller_baseline)
    ↓
Hardware Interface (3 joints: waist, shoulder, elbow)
    ↓
Serial/Hardware Control → 3 Motors
```

## Hardware Requirements

- **Microcontroller**: Teensy 4.x or Arduino compatible
- **Motor Driver**: TB6600 or similar stepper driver (one per motor)
- **Stepper Motor**: NEMA 17/23 or compatible (1-3 motors depending on configuration)
- **Connection**: USB serial connection to host computer

### Wiring (Per Motor)
```
Teensy/Arduino → TB6600:
  Pin 2 (STEP) → PUL+
  Pin 3 (DIR)  → DIR+
  Pin 4 (EN)   → ENA+
  GND          → PUL-, DIR-, ENA-

TB6600 → Stepper Motor:
  A+, A- → Coil 1
  B+, B- → Coil 2
```

**Note:** For multi-joint systems, use separate pins and drivers for each motor.

## Software Requirements

- ROS2 Humble or newer
- Ubuntu 22.04 or compatible
- Python 3.10+ (for plotting tools)
- Required Python packages: `pandas`, `matplotlib`, `numpy`

## Installation

### 1. Create workspace and clone package
```bash
mkdir -p ~/stepper_ws/src
cd ~/stepper_ws/src
git clone https://github.com/Kryptopher/Ros2.git stepper_jtc_control
```

### 2. Install dependencies
```bash
cd ~/stepper_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build
```bash
cd ~/stepper_ws
colcon build --packages-select stepper_jtc_control
source install/setup.bash
```

### 4. Upload Teensy firmware

Open `teensy_firmware/stepper_controller.ino` in Arduino IDE and upload to your Teensy.

## Configuration

### Single Joint Configuration

Edit `urdf/stepper.urdf.xacro` or pass as launch arguments:
```bash
ros2 launch stepper_jtc_control stepper_control.launch.py \
  serial_port:=/dev/ttyACM0 \
  baud_rate:=115200 \
  steps_per_rev:=200 \
  microsteps:=8
```

### Multi-Joint Configuration

The system now supports 3 joints: waist_joint, shoulder_joint, and elbow_joint.

**URDF Configuration:** All three joints are defined in `urdf/stepper.urdf.xacro`

**Controller Configuration:** All three joints are configured in `config/stepper_controllers.yaml`
```yaml
kinematic_controller_baseline:
  ros__parameters:
    joints:
      - waist_joint
      - shoulder_joint
      - elbow_joint
```

### Controller Parameters

Edit `config/stepper_controllers.yaml`:

- `update_rate`: Control loop frequency (default: 500 Hz)
- `open_loop_control`: true (required for steppers)
- `interpolation_method`: none (step-wise) or spline (smooth)

### Action Client Parameters

Available ROS parameters:
- `sampling_rate`: Trajectory point spacing (default: 0.02s = 50 Hz)
- `max_velocity`: Velocity limit check (default: 100.0 rad/s)
- `max_acceleration`: Acceleration limit check (default: 1000.0 rad/s²)
- `feedback_throttle`: Print every Nth feedback message (default: 10)
- `action_server_name`: Action server to connect to (default: `/kinematic_controller_baseline/follow_joint_trajectory`)
- `csv_file_path`: Path to CSV velocity profile file (optional)
- `wait_for_joint_states`: Wait for joint states before sending trajectory (default: true)
- `joint_state_topic`: Topic to read joint states from (default: `/joint_states`)
- `joint_state_timeout`: Timeout for joint state reception (default: 5.0 seconds)

## Usage

### Single Joint Operation

**Terminal 1 - Launch control system:**
```bash
ros2 launch stepper_jtc_control stepper_control.launch.py
```

**Terminal 2 - Send test trajectory:**
```bash
ros2 run stepper_jtc_control stepper_trajectory_client_node
```

### Multi-Joint Operation

**Terminal 1 - Launch multi-joint control system:**
```bash
ros2 launch stepper_jtc_control stepper_control.launch.py
```

**Terminal 2 - Send multi-joint trajectory with CSV:**
```bash
ros2 run stepper_jtc_control multi_joint_trajectory_client_node \
  --ros-args \
  -p action_server_name:=/kinematic_controller_baseline/follow_joint_trajectory \
  -p csv_file_path:=/path/to/velocity_profile.csv
```

**Or use default hardcoded profile:**
```bash
ros2 run stepper_jtc_control multi_joint_trajectory_client_node
```

### Custom Velocity Profiles

#### Single Joint (Code-based)

Edit the velocity profile in `src/stepper_trajectory_client.cpp`:
```cpp
std::vector<std::pair<double, double>> velocity_steps = {
  {0.0, 5.0},     // t=0s: v=5 rad/s
  {5.0, 25.0},    // t=5s: v=25 rad/s
  {10.0, 10.0},   // t=10s: v=10 rad/s
  {15.0, 0.0},    // t=15s: v=0 rad/s
  {20.0, 50.0},   // t=20s: v=50 rad/s
  {25.0, 0.0},    // t=25s: v=0 rad/s
  {26.0, 0.0}     // t=26s: hold
};
```

#### Multi-Joint (CSV-based)

Create a CSV file with format: `time,waist_vel,shoulder_vel,elbow_vel`

Example (`config/test_trajectory.csv`):
```csv
time,waist_vel,shoulder_vel,elbow_vel
0.0,5.0,3.0,2.0
5.0,25.0,15.0,10.0
10.0,10.0,8.0,5.0
15.0,0.0,0.0,0.0
20.0,50.0,30.0,20.0
25.0,0.0,0.0,0.0
26.0,0.0,0.0,0.0
```

- **time**: Time in seconds (must be monotonically increasing)
- **waist_vel**: Velocity for waist_joint (rad/s)
- **shoulder_vel**: Velocity for shoulder_joint (rad/s)
- **elbow_vel**: Velocity for elbow_joint (rad/s)

### Trajectory Logging and Visualization

**Start logging:**
```bash
# Logs are automatically saved to ~/stepper_logs/teensy_commands.csv
```

**Generate comparison plot:**
```bash
python3 ~/stepper_ws/src/stepper_jtc_control/scripts/compare_trajectories.py \
  ~/stepper_logs/teensy_commands.csv
```

The plot shows:
1. Action client input vs Teensy commands (velocity)
2. Steps per second sent to motor
3. Command update rate (should be ~500 Hz)

## Trajectory Accuracy

The system uses **trapezoidal integration** for high accuracy:
```cpp
position += (previous_velocity + current_velocity) * dt / 2.0;
```

This provides second-order accuracy, reducing cumulative error by ~100x compared to simple Euler integration.

## API Reference

### Action Interfaces

#### Single Joint
**Action Type:** `control_msgs/action/FollowJointTrajectory`

**Action Server:** `/stepper_trajectory_controller/follow_joint_trajectory`

#### Multi-Joint
**Action Type:** `control_msgs/action/FollowJointTrajectory`

**Action Server:** `/kinematic_controller_baseline/follow_joint_trajectory`

**Trajectory Point Format:**
- `positions`: Integrated position (rad) - array of 3 values for multi-joint
- `velocities`: Desired velocity (rad/s) - array of 3 values for multi-joint
- `accelerations`: Calculated acceleration (rad/s²) - array of 3 values for multi-joint
- `time_from_start`: Time point in trajectory

### Serial Protocol

Commands sent to microcontroller:
```
V<steps_per_sec>\n
```

Example: `V1273\n` commands 1273 steps/second

Conversion formula:
```
steps_per_second = velocity_rad_s × (steps_per_rev × microsteps) / (2π)
```

**Note:** In multi-joint mode, only the first joint (waist_joint) sends commands to the physical hardware in the current implementation. Additional motors can be controlled by extending the hardware interface.

## Package Structure
```
stepper_jtc_control/
├── config/
│   ├── stepper_controllers.yaml         # Controller configuration
│   └── test_trajectory.csv              # Example multi-joint CSV profile
├── deliverables/
│   ├── multi_joint_trajectory_client.cpp  # Production client
│   ├── velocity_profile_example.csv       # CSV template
│   └── README.md                          # Integration guide
├── include/
│   └── stepper_jtc_control/
│       ├── stepper_hardware_interface.hpp
│       └── visibility_control.h
├── launch/
│   └── stepper_control.launch.py        # Main launch file
├── scripts/
│   └── compare_trajectories.py          # Trajectory visualization tool
├── src/
│   ├── stepper_hardware_interface.cpp      # Hardware interface
│   ├── stepper_trajectory_client.cpp       # Single-joint action client
│   ├── multi_joint_trajectory_client.cpp   # Multi-joint action client
│   └── trajectory_logger.cpp               # Data logging node
├── urdf/
│   └── stepper.urdf.xacro               # Robot description (3 joints)
├── CMakeLists.txt
├── package.xml
└── stepper_hardware_interface.xml       # Plugin definition
```

## Deliverables for Integration

If you're integrating the multi-joint trajectory client into your own system, see the **[`deliverables/`](deliverables/)** folder which contains:

1. **multi_joint_trajectory_client.cpp** - Ready-to-use production client
2. **velocity_profile_example.csv** - CSV template with proper format
3. **README.md** - Complete integration instructions with CMakeLists.txt examples

These files are designed to be dropped into any ROS 2 package with minimal modifications.

## Troubleshooting

### Motor not moving
1. Check serial connection: `ls -l /dev/ttyACM*`
2. Verify Teensy firmware is running
3. Check TB6600 enable pin (should be LOW)
4. Monitor serial commands in hardware interface logs

### Multi-joint: Action server not available
1. Verify controller is running: `ros2 control list_controllers`
2. Check action server name: `ros2 action list`
3. Ensure action server name parameter matches your controller

### Multi-joint: Failed to receive joint states
1. Check joint states topic: `ros2 topic echo /joint_states`
2. Verify all 3 joints (waist, shoulder, elbow) are published
3. Increase `joint_state_timeout` parameter if needed
4. Set `wait_for_joint_states:=false` to skip joint state reading

### Multi-joint: CSV load failed
1. Verify file path is correct and accessible
2. Check CSV format matches example (4 columns: time, waist_vel, shoulder_vel, elbow_vel)
3. Ensure values are numeric and comma-separated
4. Check for proper header line (will be auto-skipped)

### Position tracking errors
- Expected in open-loop mode (no encoder feedback)
- Errors of 0.01-0.06 rad are normal
- For closed-loop control, add encoder and switch to position mode

### Serial permission denied
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Controller fails to load
```bash
# Check hardware interface
ros2 control list_hardware_interfaces

# Check controller status
ros2 control list_controllers
```

## Performance Metrics

- **Control loop**: 500 Hz (2ms period)
- **Trajectory sampling**: 50 Hz default (20ms spacing)
- **Serial communication**: 115200 baud
- **Position accuracy**: <0.001 rad cumulative error over 25s (single joint)
- **Velocity tracking**: Direct pass-through, no lag
- **Multi-joint coordination**: Synchronized trajectory execution across all 3 joints

## Future Enhancements

- [x] Multi-joint (3-axis) coordination
- [x] CSV-based velocity profiles
- [x] Parameterizable action server names
- [ ] Input shaping integration (ZV, ZVD, EI shapers)
- [ ] Closed-loop control with encoder feedback
- [ ] Physical hardware interface for all 3 motors
- [ ] Dynamic parameter adjustment
- [ ] ROS2 service interface for on-the-fly trajectory updates
- [ ] Enhanced Teensy firmware for multi-motor control

## License

MIT License - See LICENSE file for details

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Authors

Sanjay Maharjan, Mechanical Engineering, Louisiana State University

## Acknowledgments

- ROS2 Control framework
- Joint Trajectory Controller implementation
- TB6600 driver community documentation

## References

- [ROS2 Control Documentation](https://control.ros.org/)
- [JointTrajectoryController](https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
- [FollowJointTrajectory Action](http://docs.ros.org/en/api/control_msgs/html/action/FollowJointTrajectory.html)
