#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <array>
#include <limits>
#include <thread>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

// Global pointer for signal handler
static std::shared_ptr<rclcpp::Node> g_node_ptr = nullptr;
static rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr g_publisher_ptr = nullptr;

// Signal handler to send zero velocities before shutdown
void signal_handler(int signum)
{
  if (g_publisher_ptr && g_node_ptr) {
    RCLCPP_INFO(g_node_ptr->get_logger(), "Signal caught - sending zero velocities");
    
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {0.0, 0.0, 0.0};
    
    // Publish multiple times to ensure it gets through
    for (int i = 0; i < 3; i++) {
      g_publisher_ptr->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  
  // Now let ROS handle the shutdown
  rclcpp::shutdown();
  exit(signum);
}

// Stores a single velocity command with timestamp and motor velocities
struct VelocityCommand
{
  double time;
  std::array<double, 3> velocities;  // [waist, shoulder, elbow] in motor rad/s
};

class CSVVelocityPublisher : public rclcpp::Node
{
public:
  CSVVelocityPublisher()
  : Node("csv_velocity_publisher"), current_index_(0), finished_(false)
  {
    // Declare configuration parameters
    this->declare_parameter("csv_file_path", "");
    this->declare_parameter("command_topic", "/forward_command_controller/commands");
    this->declare_parameter("max_waist_velocity", 200.0);
    this->declare_parameter("max_shoulder_velocity", 1200.0);
    this->declare_parameter("max_elbow_velocity", 1200.0);
    
    // Retrieve parameters
    csv_file_path_ = this->get_parameter("csv_file_path").as_string();
    command_topic_ = this->get_parameter("command_topic").as_string();
    max_velocities_[0] = this->get_parameter("max_waist_velocity").as_double();
    max_velocities_[1] = this->get_parameter("max_shoulder_velocity").as_double();
    max_velocities_[2] = this->get_parameter("max_elbow_velocity").as_double();
    
    // Validate that CSV file path was provided
    if (csv_file_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No CSV file path provided");
      rclcpp::shutdown();
      return;
    }
    
    // Load velocity profile from CSV
    if (!load_csv()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load CSV file");
      rclcpp::shutdown();
      return;
    }
    
    // Validate all velocities are within limits
    if (!validate_velocities()) {
      RCLCPP_ERROR(this->get_logger(), "Velocity validation failed");
      rclcpp::shutdown();
      return;
    }
    
    // Calculate publish rate statistics from CSV
    calculate_publish_rates();
    
    // Create publisher for motor commands
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);
  
    // Log initialization summary
    RCLCPP_INFO(this->get_logger(), "CSV Velocity Publisher initialized");
    RCLCPP_INFO(this->get_logger(), "  File: %s", csv_file_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Commands: %zu", commands_.size());
    RCLCPP_INFO(this->get_logger(), "  Duration: %.2f seconds", commands_.back().time);
    RCLCPP_INFO(this->get_logger(), "  Topic: %s", command_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Publish rate: %.1f - %.1f Hz (step-wise)", 
                min_publish_rate_, max_publish_rate_);
    RCLCPP_INFO(this->get_logger(), "  Timer rate: %.0f Hz (auto-calculated)", timer_rate_);
    RCLCPP_INFO(this->get_logger(), "Motor limits: waist=±%.0f, shoulder/elbow=±%.0f rad/s",
                max_velocities_[0], max_velocities_[1]);
    
    // Send initial zero command
    publish_velocity({0.0, 0.0, 0.0});
    
    // Start periodic publishing timer with auto-calculated rate
    auto period = std::chrono::duration<double>(1.0 / timer_rate_);
    timer_ = this->create_wall_timer(period, 
      [this]() { timer_callback(); });
    
    start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Starting velocity profile...");
  }

  // Getter for publisher (to use in signal handler)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr get_publisher() 
  {
    return publisher_;
  }

private:
  bool load_csv()
  {
    std::ifstream file(csv_file_path_);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", csv_file_path_.c_str());
      return false;
    }
    
    std::string line;
    int line_number = 0;
    bool header_skipped = false;
    
    while (std::getline(file, line)) {
      line_number++;
      
      // Skip empty lines
      if (line.empty() || line.find_first_not_of(" \t\r\n") == std::string::npos) {
        continue;
      }
      
      // Skip header if detected
      if (!header_skipped && (line.find("time") != std::string::npos || 
                               line.find("waist") != std::string::npos)) {
        header_skipped = true;
        continue;
      }
      
      // Parse comma-separated values
      std::vector<double> values;
      std::stringstream ss(line);
      std::string token;
      
      while (std::getline(ss, token, ',')) {
        // Trim whitespace
        token.erase(0, token.find_first_not_of(" \t\r\n"));
        token.erase(token.find_last_not_of(" \t\r\n") + 1);
        
        // Convert to double
        try {
          values.push_back(std::stod(token));
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Parse error on line %d: %s", line_number, e.what());
          return false;
        }
      }
      
      // Validate expected format: time, waist, shoulder, elbow
      if (values.size() != 4) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Expected 4 values on line %d, got %zu", line_number, values.size());
        return false;
      }
      
      // Create command from parsed values
      VelocityCommand cmd;
      cmd.time = values[0];
      cmd.velocities[0] = values[1];  // waist
      cmd.velocities[1] = values[2];  // shoulder
      cmd.velocities[2] = values[3];  // elbow
      
      commands_.push_back(cmd);
    }
    
    file.close();
    
    if (commands_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No valid commands found in CSV");
      return false;
    }
    
    // Verify timestamps are strictly increasing
    for (size_t i = 1; i < commands_.size(); i++) {
      if (commands_[i].time <= commands_[i-1].time) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Time values must increase (error between lines %zu and %zu)", i, i+1);
        return false;
      }
    }
    
    return true;
  }

  bool validate_velocities()
  {
    const std::array<std::string, 3> names = {"waist", "shoulder", "elbow"};
    
    for (size_t cmd_index = 0; cmd_index < commands_.size(); cmd_index++) {
      for (int motor_index = 0; motor_index < 3; motor_index++) {
        if (std::abs(commands_[cmd_index].velocities[motor_index]) > max_velocities_[motor_index]) {
          RCLCPP_ERROR(this->get_logger(),
                       "%s motor velocity %.0f exceeds limit ±%.0f at t=%.2f",
                       names[motor_index].c_str(),
                       commands_[cmd_index].velocities[motor_index],
                       max_velocities_[motor_index],
                       commands_[cmd_index].time);
          return false;
        }
      }
    }
    
    return true;
  }

  void calculate_publish_rates()
  {
    // Find min and max time intervals in CSV
    double min_interval = std::numeric_limits<double>::max();
    double max_interval = 0.0;
    
    for (size_t cmd_index = 1; cmd_index < commands_.size(); cmd_index++) {
      double interval = commands_[cmd_index].time - commands_[cmd_index-1].time;
      if (interval < min_interval) min_interval = interval;
      if (interval > max_interval) max_interval = interval;
    }
    
    // Calculate publish rate range
    max_publish_rate_ = 1.0 / min_interval;
    min_publish_rate_ = 1.0 / max_interval;
    
    // Set timer rate to 10x the fastest publish rate to ensure accurate timing
    // This guarantees we never miss a timestamp
    timer_rate_ = 10.0 / min_interval;
    
    // Cap timer rate at reasonable maximum to avoid excessive CPU usage
    if (timer_rate_ > 1000.0) {
      RCLCPP_WARN(this->get_logger(), 
                  "Timer rate would be %.0f Hz, capping at 1000 Hz", timer_rate_);
      timer_rate_ = 1000.0;
    }
  }

  void timer_callback()
  {
    double elapsed = (this->now() - start_time_).seconds();
    
    // Check if it's time for the next command (step-wise, no interpolation)
    if (current_index_ < commands_.size()) {
      if (elapsed >= commands_[current_index_].time) {
        // Publish new velocity step
        publish_velocity(commands_[current_index_].velocities);
        
        RCLCPP_INFO(this->get_logger(), 
                    "t=%.2fs: [%.0f, %.0f, %.0f] rad/s",
                    elapsed,
                    commands_[current_index_].velocities[0],
                    commands_[current_index_].velocities[1],
                    commands_[current_index_].velocities[2]);
        
        current_index_++;
      }
    } else if (!finished_) {
      // Profile complete - send zero and stop
      publish_velocity({0.0, 0.0, 0.0});
      RCLCPP_INFO(this->get_logger(), "Profile complete");
      timer_->cancel();
      finished_ = true;
    }
  }

  void publish_velocity(const std::array<double, 3>& vel)
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {vel[0], vel[1], vel[2]};
    publisher_->publish(msg);
  }

  // ROS components
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Configuration
  std::string csv_file_path_;
  std::string command_topic_;
  std::array<double, 3> max_velocities_;
  
  // Runtime state
  std::vector<VelocityCommand> commands_;
  size_t current_index_;
  rclcpp::Time start_time_;
  bool finished_;
  
  // Auto-calculated rates
  double timer_rate_;
  double min_publish_rate_;
  double max_publish_rate_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CSVVelocityPublisher>();
  
  // Set global pointers for signal handler (do it here, not in constructor)
  g_node_ptr = node;
  g_publisher_ptr = node->get_publisher();
  
  // Register signal handlers
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  
  rclcpp::spin(node);
  return 0;
}