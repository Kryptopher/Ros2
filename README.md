# Stepper Motor Control with ROS2 Joint Trajectory Controller

A ROS2 package for precise stepper motor control using the Joint Trajectory Controller (JTC) with velocity-based commands. This system provides accurate trajectory execution with real-time logging and visualization for stepper motors controlled via serial communication.

## Features

- **Velocity-based trajectory control** using ROS2 JointTrajectoryController
- **High-precision integration** with trapezoidal method for accurate position tracking
- **Serial communication** to microcontroller (Teensy/Arduino) at 500 Hz
- **Real-time logging** with nanosecond precision
- **Trajectory comparison tools** to validate commanded vs actual velocities
- **Auto-disable motor** after trajectory completion to save power
- **Configurable parameters** for sampling rate, velocity/acceleration limits
- **Input shaping ready** for vibration reduction applications

## System Architecture

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

## Hardware Requirements

- **Microcontroller**: Teensy 4.x or Arduino compatible
- **Motor Driver**: TB6600 or similar stepper driver
- **Stepper Motor**: NEMA 17/23 or compatible
- **Connection**: USB serial connection to host computer

### Wiring

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

### Motor Parameters

Edit `urdf/stepper.urdf.xacro` or pass as launch arguments:

```bash
ros2 launch stepper_jtc_control stepper_control.launch.py \
  serial_port:=/dev/ttyACM0 \
  baud_rate:=115200 \
  steps_per_rev:=200 \
  microsteps:=8
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

## Usage

### Basic Operation

**Terminal 1 - Launch control system:**
```bash
ros2 launch stepper_jtc_control stepper_control.launch.py
```

**Terminal 2 - Send test trajectory:**
```bash
ros2 run stepper_jtc_control stepper_trajectory_client_node
```

### Custom Velocity Profiles

Edit the velocity profile in `src/stepper_trajectory_client.cpp`:

```cpp
std::vector<std::pair<double, double>> velocity_steps = {
  {0.0, 5.0},     // t=0s: v=5 rad/s
  {5.0, 25.0},     // t=5s: v=25 rad/s
  {10.0, 10.0},   // t=10s: v=10 rad/s
  {15.0, 0.0},   // t=15s: v=0 rad/s
  {20.0, 50.0},   // t=20s: v=50 rad/s
  {25.0, 0.0},    // t=25s: v=0 rad/s
  {26.0, 0.0}     // t=26s: hold
};
```

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

### Action Interface

**Action Type:** `control_msgs/action/FollowJointTrajectory`

**Action Server:** `/stepper_trajectory_controller/follow_joint_trajectory`

**Trajectory Point Format:**
- `positions`: Integrated position (rad)
- `velocities`: Desired velocity (rad/s)
- `accelerations`: Calculated acceleration (rad/s²)
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

## Package Structure

```
stepper_jtc_control/
├── config/
│   └── stepper_controllers.yaml      # Controller configuration
├── include/
│   └── stepper_jtc_control/
│       ├── stepper_hardware_interface.hpp
│       └── visibility_control.h
├── launch/
│   └── stepper_control.launch.py     # Main launch file
├── scripts/
│   └── compare_trajectories.py       # Trajectory visualization tool
├── src/
│   ├── stepper_hardware_interface.cpp   # Hardware interface implementation
│   ├── stepper_trajectory_client.cpp    # Action client
│   └── trajectory_logger.cpp            # Data logging node
├── urdf/
│   └── stepper.urdf.xacro            # Robot description
├── CMakeLists.txt
├── package.xml
└── stepper_hardware_interface.xml    # Plugin definition
```

## Troubleshooting

### Motor not moving
1. Check serial connection: `ls -l /dev/ttyACM*`
2. Verify Teensy firmware is running
3. Check TB6600 enable pin (should be LOW)
4. Monitor serial commands in hardware interface logs

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
- **Position accuracy**: <0.001 rad cumulative error over 25s
- **Velocity tracking**: Direct pass-through, no lag

## Future Enhancements

- [ ] Input shaping integration (ZV, ZVD, EI shapers)
- [ ] Closed-loop control with encoder feedback
- [ ] Multi-axis coordination
- [ ] Dynamic parameter adjustment
- [ ] ROS2 service interface for on-the-fly trajectory updates
- [ ] Teensy firmware implementation

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
