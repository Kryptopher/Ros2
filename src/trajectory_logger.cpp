#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <fstream>
#include <vector>
#include <string>

class TrajectoryLogger : public rclcpp::Node
{
public:
  TrajectoryLogger() : Node("trajectory_logger")
  {
    // Subscribe to controller state
    state_sub_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/stepper_trajectory_controller/controller_state",
      10,
      std::bind(&TrajectoryLogger::state_callback, this, std::placeholders::_1));
    
    // Parameters
    this->declare_parameter("log_directory", "/home/sanjay/stepper_logs");
    this->declare_parameter("auto_plot", true);
    
    log_dir_ = this->get_parameter("log_directory").as_string();
    auto_plot_ = this->get_parameter("auto_plot").as_bool();
    
    // Create log directory
    std::string mkdir_cmd = "mkdir -p " + log_dir_;
    int result = system(mkdir_cmd.c_str());
    (void)result;  // Suppress unused warning
    
    // Generate timestamp for this session
    auto now = this->now();
    timestamp_ = std::to_string(now.nanoseconds());
    
    log_file_path_ = log_dir_ + "/trajectory_" + timestamp_ + ".csv";
    
    // Open log file
    log_file_.open(log_file_path_);
    log_file_ << "time,desired_position,actual_position,error_position,"
              << "desired_velocity,actual_velocity,error_velocity\n";
    
    RCLCPP_INFO(this->get_logger(), "Trajectory logger started");
    RCLCPP_INFO(this->get_logger(), "Logging to: %s", log_file_path_.c_str());
    
    logging_active_ = true;
    start_time_ = this->now();
  }
  
  ~TrajectoryLogger()
  {
    if (log_file_.is_open()) {
      log_file_.close();
      RCLCPP_INFO(this->get_logger(), "Log file closed: %s", log_file_path_.c_str());
      RCLCPP_INFO(this->get_logger(), "Total data points logged: %zu", data_points_);
      
      if (auto_plot_ && data_points_ > 0) {
        generate_plots();
      }
    }
  }

private:
  void state_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    if (!logging_active_ || !log_file_.is_open()) return;
    
    // Calculate elapsed time
    double elapsed = (this->now() - start_time_).seconds();
    
    // Log data for first joint (stepper_joint)
    if (!msg->joint_names.empty() && 
    !msg->reference.positions.empty() &&
    !msg->feedback.positions.empty() &&
    !msg->error.positions.empty() &&
    !msg->reference.velocities.empty() &&
    !msg->feedback.velocities.empty() &&
    !msg->error.velocities.empty()) {
  
  log_file_ << elapsed << ","
            << msg->reference.positions[0] << ","
            << msg->feedback.positions[0] << ","
            << msg->error.positions[0] << ","
            << msg->reference.velocities[0] << ","
            << msg->feedback.velocities[0] << ","
            << msg->error.velocities[0] << "\n";  
      data_points_++;
      
      // Print progress every 100 points
      if (data_points_ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "Logged %zu data points (t=%.2fs)", 
                    data_points_, elapsed);
      }
    }
  }
  
  void generate_plots()
  {
    RCLCPP_INFO(this->get_logger(), "Generating plots...");
    
    // Create Python plotting script
    std::string plot_script_path = log_dir_ + "/plot_trajectory.py";
    std::ofstream plot_script(plot_script_path);
    
    plot_script << R"(#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import sys

if len(sys.argv) < 2:
    print("Usage: python3 plot_trajectory.py <csv_file>")
    sys.exit(1)

# Read CSV file
csv_file = sys.argv[1]
df = pd.read_csv(csv_file)

print(f"Loaded {len(df)} data points")

# Create figure with subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
fig.suptitle('Trajectory Tracking Analysis', fontsize=16)

# Plot 1: Position tracking
ax1.plot(df['time'], df['desired_position'], 'b-', label='Desired Position', linewidth=2)
ax1.plot(df['time'], df['actual_position'], 'r--', label='Actual Position', linewidth=1.5)
ax1.set_ylabel('Position (rad)')
ax1.set_title('Position Tracking')
ax1.legend()
ax1.grid(True, alpha=0.3)

# Plot 2: Velocity tracking
ax2.plot(df['time'], df['desired_velocity'], 'b-', label='Desired Velocity', linewidth=2)
ax2.plot(df['time'], df['actual_velocity'], 'r--', label='Actual Velocity', linewidth=1.5)
ax2.set_ylabel('Velocity (rad/s)')
ax2.set_title('Velocity Tracking')
ax2.legend()
ax2.grid(True, alpha=0.3)

# Plot 3: Tracking errors
ax3.plot(df['time'], df['error_position'], 'g-', label='Position Error', linewidth=1.5)
ax3_twin = ax3.twinx()
ax3_twin.plot(df['time'], df['error_velocity'], 'orange', label='Velocity Error', linewidth=1.5)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Position Error (rad)', color='g')
ax3_twin.set_ylabel('Velocity Error (rad/s)', color='orange')
ax3.set_title('Tracking Errors')
ax3.tick_params(axis='y', labelcolor='g')
ax3_twin.tick_params(axis='y', labelcolor='orange')
ax3.grid(True, alpha=0.3)
ax3.legend(loc='upper left')
ax3_twin.legend(loc='upper right')

plt.tight_layout()

# Save plot
output_file = csv_file.replace('.csv', '.png')
plt.savefig(output_file, dpi=150, bbox_inches='tight')
print(f"Plot saved to: {output_file}")

# Show plot
plt.show()
)";
    
    plot_script.close();
    
    // Make script executable
    std::string chmod_cmd = "chmod +x " + plot_script_path;
    int result = system(chmod_cmd.c_str());
    (void)result;
    
    // Run plotting script
    std::string plot_cmd = "python3 " + plot_script_path + " " + log_file_path_ + " &";
    RCLCPP_INFO(this->get_logger(), "Running: %s", plot_cmd.c_str());
    result = system(plot_cmd.c_str());
    (void)result;
    
    RCLCPP_INFO(this->get_logger(), "Plot will be saved to: %s", 
                (log_file_path_.substr(0, log_file_path_.length()-4) + ".png").c_str());
  }

  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_sub_;
  std::ofstream log_file_;
  std::string log_dir_;
  std::string log_file_path_;
  std::string timestamp_;
  bool auto_plot_;
  bool logging_active_;
  rclcpp::Time start_time_;
  size_t data_points_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryLogger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
