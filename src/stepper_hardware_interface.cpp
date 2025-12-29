#include "stepper_jtc_control/stepper_hardware_interface.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace stepper_jtc_control
{

hardware_interface::CallbackReturn StepperHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize state storage
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  // Parse hardware parameters
  serial_port_ = info_.hardware_parameters["serial_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  steps_per_rev_ = std::stoi(info_.hardware_parameters["steps_per_rev"]);
  microsteps_ = std::stoi(info_.hardware_parameters["microsteps"]);

  // Verify joint configuration
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Verify command interface
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("StepperHardwareInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", 
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("StepperHardwareInterface"),
        "Joint '%s' has %s command interface. '%s' expected.", 
        joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Verify state interfaces
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("StepperHardwareInterface"),
        "Joint '%s' has %zu state interfaces. 2 expected.", 
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StepperHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("StepperHardwareInterface"), "Configuring...");
  
  // Open serial port
  if (!open_serial_port())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("StepperHardwareInterface"), 
              "Successfully configured on %s", serial_port_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> 
StepperHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, 
        hardware_interface::HW_IF_POSITION, 
        &hw_positions_[i]));
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, 
        hardware_interface::HW_IF_VELOCITY, 
        &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
StepperHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, 
        hardware_interface::HW_IF_VELOCITY, 
        &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn StepperHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("StepperHardwareInterface"), "Activating...");

  // Initialize commands to zero
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    hw_commands_[i] = 0.0;
    hw_velocities_[i] = 0.0;
  }

  // Send stop command to motor
  send_velocity_command(0.0);

  RCLCPP_INFO(rclcpp::get_logger("StepperHardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StepperHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("StepperHardwareInterface"), "Deactivating...");

  // Stop the motor
  send_velocity_command(0.0);
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type StepperHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Open-loop state estimation
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    // Update velocity state (echo back commanded velocity)
    hw_velocities_[i] = hw_commands_[i];
    
    // Integrate velocity to get position
    hw_positions_[i] += hw_velocities_[i] * period.seconds();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type StepperHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Send velocity command to Teensy
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    send_velocity_command(hw_commands_[i]);
  }

  return hardware_interface::return_type::OK;
}

// Private helper functions

bool StepperHardwareInterface::open_serial_port()
{
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("StepperHardwareInterface"),
                 "Failed to open serial port: %s", serial_port_.c_str());
    return false;
  }

  // Configure serial port
  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("StepperHardwareInterface"),
                 "Error from tcgetattr");
    close(serial_fd_);
    return false;
  }

  // Set baud rate
  speed_t baud;
  switch (baud_rate_)
  {
    case 9600: baud = B9600; break;
    case 19200: baud = B19200; break;
    case 38400: baud = B38400; break;
    case 57600: baud = B57600; break;
    case 115200: baud = B115200; break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("StepperHardwareInterface"),
                   "Unsupported baud rate: %d", baud_rate_);
      close(serial_fd_);
      return false;
  }

  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);

  // 8N1 mode
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 10;
  tty.c_cc[VMIN] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("StepperHardwareInterface"),
                 "Error from tcsetattr");
    close(serial_fd_);
    return false;
  }

  // Flush any old data
  tcflush(serial_fd_, TCIOFLUSH);

  RCLCPP_INFO(rclcpp::get_logger("StepperHardwareInterface"),
              "Serial port opened: %s at %d baud", 
              serial_port_.c_str(), baud_rate_);

  return true;
}

void StepperHardwareInterface::close_serial_port()
{
  if (serial_fd_ >= 0)
  {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool StepperHardwareInterface::write_serial(const std::string& data)
{
  if (serial_fd_ < 0)
  {
    return false;
  }

  ssize_t bytes_written = ::write(serial_fd_, data.c_str(), data.length());
  
  if (bytes_written < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("StepperHardwareInterface"),
                 "Failed to write to serial port");
    return false;
  }

  return true;
}

void StepperHardwareInterface::send_velocity_command(double velocity_rad_s)
{
  // Convert rad/s to steps/s
  double steps_per_rad = (steps_per_rev_ * microsteps_) / (2.0 * M_PI);
  int32_t steps_per_sec = static_cast<int32_t>(velocity_rad_s * steps_per_rad);
  
  // Format: "V<steps_per_sec>\n"
  std::string command = "V" + std::to_string(steps_per_sec) + "\n";
  
  // LOG WHAT WE'RE SENDING TO TEENSY
  static std::ofstream teensy_log;
  static bool log_initialized = false;
  static bool trajectory_started = false;
  static rclcpp::Time log_start_time;
  
  if (!log_initialized) {
    std::string mkdir_cmd = "mkdir -p /home/sanjay/stepper_logs";
    int result = system(mkdir_cmd.c_str());
    (void)result;
    
    std::string log_path = "/home/sanjay/stepper_logs/teensy_commands.csv";
    teensy_log.open(log_path);
    teensy_log << "time,velocity_rad_s,steps_per_sec\n";
    log_initialized = true;
    RCLCPP_INFO(rclcpp::get_logger("StepperHardwareInterface"),
                "Teensy command logging started: %s", log_path.c_str());
  }
  
  // Start timing when first non-zero velocity is commanded
  if (!trajectory_started && std::abs(velocity_rad_s) > 0.01) {
    log_start_time = rclcpp::Clock().now();
    trajectory_started = true;
    RCLCPP_INFO(rclcpp::get_logger("StepperHardwareInterface"),
                "Trajectory execution started - resetting log timer");
  }
  
  // Only log if trajectory has started
if (trajectory_started) {
  int64_t elapsed_ns = (rclcpp::Clock().now() - log_start_time).nanoseconds();
  double elapsed = elapsed_ns / 1e9;  // Convert to seconds with nanosecond precision
  teensy_log << std::fixed << std::setprecision(9) << elapsed << "," 
             << std::setprecision(6) << velocity_rad_s << "," 
             << steps_per_sec << "\n";
  teensy_log.flush();
} 
  // Send to Teensy
  if (!write_serial(command))
  {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("StepperHardwareInterface"),
      *rclcpp::Clock::make_shared(),
      5000,
      "Failed to send velocity command to Teensy");
  }
}

}  // namespace stepper_jtc_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  stepper_jtc_control::StepperHardwareInterface, 
  hardware_interface::SystemInterface)
