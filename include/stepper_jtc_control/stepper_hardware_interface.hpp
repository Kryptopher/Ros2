#ifndef STEPPER_JTC_CONTROL__STEPPER_HARDWARE_INTERFACE_HPP_
#define STEPPER_JTC_CONTROL__STEPPER_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "stepper_jtc_control/visibility_control.h"

namespace stepper_jtc_control
{
class StepperHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(StepperHardwareInterface)

  STEPPER_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  STEPPER_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  STEPPER_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  STEPPER_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  STEPPER_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  STEPPER_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  STEPPER_CONTROL_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  STEPPER_CONTROL_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial communication
  int serial_fd_;
  std::string serial_port_;
  int baud_rate_;
  
  // Motor parameters
  int steps_per_rev_;
  int microsteps_;
  
  // Joint state storage
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  
  // Helper functions
  bool open_serial_port();
  void close_serial_port();
  bool write_serial(const std::string& data);
  void send_velocity_command(double velocity_rad_s);
};

}  // namespace stepper_jtc_control

#endif  // STEPPER_JTC_CONTROL__STEPPER_HARDWARE_INTERFACE_HPP_
