#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <array>
#include <cmath>
#include <fstream>
#include <algorithm>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace stepper_jtc_control
{

struct VelocityStep
{
  double time;
  std::array<double, 3> velocities;  // waist, shoulder, elbow (rad/s)
};

class MultiJointTrajectoryClient : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  explicit MultiJointTrajectoryClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("multi_joint_trajectory_client", options)
  {
    // Declare parameters
    this->declare_parameter("action_server_name", 
      "/kinematic_controller_baseline/follow_joint_trajectory");
    this->declare_parameter("csv_file_path", "");
    this->declare_parameter("sampling_rate", 0.01);
    
    // UPDATED: Per-joint limits from MoveIt configuration
    this->declare_parameter("max_velocities", std::vector<double>{0.20, 0.035, 0.035});
    this->declare_parameter("max_accelerations", std::vector<double>{9999.0, 9999.0, 9999.0});
    
    this->declare_parameter("feedback_throttle", 10);
    this->declare_parameter("wait_for_joint_states", true);
    this->declare_parameter("joint_state_topic", "/joint_states");
    this->declare_parameter("joint_state_timeout", 5.0);
    
    // UPDATED: Position limits from MoveIt configuration
    this->declare_parameter("enforce_position_limits", true);
    this->declare_parameter("min_positions", std::vector<double>{-1.70, -1.0, -1.570796});
    this->declare_parameter("max_positions", std::vector<double>{1.17, 0.7853981, 1.0});
    
    // Get parameters
    action_server_name_ = this->get_parameter("action_server_name").as_string();
    csv_file_path_ = this->get_parameter("csv_file_path").as_string();
    sampling_rate_ = this->get_parameter("sampling_rate").as_double();
    
    // Get per-joint velocity limits
    auto max_vel_param = this->get_parameter("max_velocities").as_double_array();
    if (max_vel_param.size() != 3) {
      RCLCPP_ERROR(this->get_logger(), 
                   "max_velocities must have exactly 3 values (one per joint)");
      rclcpp::shutdown();
      return;
    }
    max_velocities_ = {max_vel_param[0], max_vel_param[1], max_vel_param[2]};
    
    // Get per-joint acceleration limits
    auto max_acc_param = this->get_parameter("max_accelerations").as_double_array();
    if (max_acc_param.size() != 3) {
      RCLCPP_ERROR(this->get_logger(), 
                   "max_accelerations must have exactly 3 values (one per joint)");
      rclcpp::shutdown();
      return;
    }
    max_accelerations_ = {max_acc_param[0], max_acc_param[1], max_acc_param[2]};
    
    feedback_throttle_ = this->get_parameter("feedback_throttle").as_int();
    wait_for_joint_states_ = this->get_parameter("wait_for_joint_states").as_bool();
    joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
    joint_state_timeout_ = this->get_parameter("joint_state_timeout").as_double();
    
    // Get position limits parameters
    enforce_position_limits_ = this->get_parameter("enforce_position_limits").as_bool();
    auto min_pos_param = this->get_parameter("min_positions").as_double_array();
    auto max_pos_param = this->get_parameter("max_positions").as_double_array();
    
    if (min_pos_param.size() != 3 || max_pos_param.size() != 3) {
      RCLCPP_ERROR(this->get_logger(), 
                   "Position limits must have exactly 3 values (one per joint)");
      rclcpp::shutdown();
      return;
    }
    
    min_positions_ = {min_pos_param[0], min_pos_param[1], min_pos_param[2]};
    max_positions_ = {max_pos_param[0], max_pos_param[1], max_pos_param[2]};
    
    // Initialize joint names in required order: waist, shoulder, elbow
    joint_names_ = {"waist_joint", "shoulder_joint", "elbow_joint"};
    
    RCLCPP_INFO(this->get_logger(), "Multi-joint trajectory client initialized");
    RCLCPP_INFO(this->get_logger(), "Action server: %s", action_server_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Joint state topic: %s", joint_state_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "CSV file: %s", 
                csv_file_path_.empty() ? "Not specified (will use default)" : csv_file_path_.c_str());
    
    // Log per-joint limits
    RCLCPP_INFO(this->get_logger(), "Joint limits (from MoveIt configuration):");
    for (size_t i = 0; i < 3; i++) {
      RCLCPP_INFO(this->get_logger(), "  %s:", joint_names_[i].c_str());
      RCLCPP_INFO(this->get_logger(), "    Position: [%.4f, %.4f] rad", 
                  min_positions_[i], max_positions_[i]);
      RCLCPP_INFO(this->get_logger(), "    Velocity: %.4f rad/s", max_velocities_[i]);
      RCLCPP_INFO(this->get_logger(), "    Acceleration: %.4f rad/s²", max_accelerations_[i]);
    }
    
    if (!enforce_position_limits_) {
      RCLCPP_WARN(this->get_logger(), "Position limits enforcement is DISABLED");
    }
    
    // Create action client
    this->client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, action_server_name_);
    
    // Subscribe to joint states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 10,
      std::bind(&MultiJointTrajectoryClient::joint_state_callback, this, std::placeholders::_1));
    
    // Start timer to send trajectory
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&MultiJointTrajectoryClient::send_trajectory, this));
  }

  // Destructor
  ~MultiJointTrajectoryClient()
  {
    if (active_goal_handle_) {
      RCLCPP_INFO(this->get_logger(), "Cancelling active trajectory before shutdown");
      auto cancel_future = client_ptr_->async_cancel_goal(active_goal_handle_);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_future, 
                                             std::chrono::milliseconds(500)) == 
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Trajectory cancelled successfully");
      }
    }
  }

  void send_trajectory()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    // Wait for joint states if required
    if (wait_for_joint_states_) {
      if (!wait_for_joint_states_with_timeout(joint_state_timeout_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive joint states within timeout");
        rclcpp::shutdown();
        return;
      }
      
      RCLCPP_INFO(this->get_logger(), "Initial joint positions received:");
      RCLCPP_INFO(this->get_logger(), "  waist_joint: %.6f rad", current_joint_positions_[0]);
      RCLCPP_INFO(this->get_logger(), "  shoulder_joint: %.6f rad", current_joint_positions_[1]);
      RCLCPP_INFO(this->get_logger(), "  elbow_joint: %.6f rad", current_joint_positions_[2]);
    }

    // Wait for action server
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    // Load velocity profile
    std::vector<VelocityStep> velocity_steps;
    
    if (csv_file_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No CSV file path provided. Set csv_file_path parameter.");
      rclcpp::shutdown();
      return;
    }
    
    velocity_steps = load_velocity_profile_from_csv(csv_file_path_);
    if (velocity_steps.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load velocity profile from CSV");
      rclcpp::shutdown();
      return;
    }

    // Validate velocity profile
    if (!validate_velocity_profile(velocity_steps)) {
      RCLCPP_ERROR(this->get_logger(), "Velocity profile validation failed");
      return;
    }

    // Create goal message
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = joint_names_;
    goal_msg.trajectory.points = generate_stepwise_trajectory(velocity_steps, sampling_rate_);
    goal_msg.trajectory.header.stamp.sec = 0;
    goal_msg.trajectory.header.stamp.nanosec = 0;
    goal_msg.trajectory.header.frame_id = "";

    // Log trajectory information
    log_trajectory_info(goal_msg, velocity_steps);

    // Send goal
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = 
    [this](auto goal_handle) { goal_response_callback(goal_handle); };
    send_goal_options.feedback_callback =
    [this](auto goal_handle, auto feedback) { feedback_callback(goal_handle, feedback); };
    send_goal_options.result_callback =
    [this](auto result) { result_callback(result); };

    feedback_counter_ = 0;
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr active_goal_handle_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::string action_server_name_;
  std::string csv_file_path_;
  std::string joint_state_topic_;
  std::vector<std::string> joint_names_;
  
  double sampling_rate_;
  std::array<double, 3> max_velocities_;      // Per-joint velocity limits
  std::array<double, 3> max_accelerations_;   // Per-joint acceleration limits
  double joint_state_timeout_;
  int feedback_throttle_;
  int feedback_counter_;
  bool wait_for_joint_states_;
  
  std::array<double, 3> current_joint_positions_{0.0, 0.0, 0.0};
  bool positions_received_{false};
  
  bool enforce_position_limits_;
  std::array<double, 3> min_positions_;
  std::array<double, 3> max_positions_;

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Extract positions for the three joints in correct order
    bool waist_found = false;
    bool shoulder_found = false;
    bool elbow_found = false;
    
    for (size_t i = 0; i < msg->name.size(); i++) {
      if (msg->name[i] == "waist_joint") {
        current_joint_positions_[0] = msg->position[i];
        waist_found = true;
      } else if (msg->name[i] == "shoulder_joint") {
        current_joint_positions_[1] = msg->position[i];
        shoulder_found = true;
      } else if (msg->name[i] == "elbow_joint") {
        current_joint_positions_[2] = msg->position[i];
        elbow_found = true;
      }
    }
    
    if (!positions_received_) {
      positions_received_ = waist_found && shoulder_found && elbow_found;
      
      if (positions_received_) {
        RCLCPP_DEBUG(this->get_logger(), "All 3 joint positions received from joint_states");
      } else {
        if (!waist_found) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                                "waist_joint not found in joint_states");
        if (!shoulder_found) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                                   "shoulder_joint not found in joint_states");
        if (!elbow_found) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                                "elbow_joint not found in joint_states");
      }
    }
  }

  bool wait_for_joint_states_with_timeout(double timeout_seconds)
  {
    auto start_time = this->now();
    rclcpp::Rate rate(100);
    
    while (rclcpp::ok()) {
      if (positions_received_) {
        return true;
      }
      
      auto elapsed = (this->now() - start_time).seconds();
      if (elapsed > timeout_seconds) {
        return false;
      }
      
      rclcpp::spin_some(this->get_node_base_interface());
      rate.sleep();
    }
    
    return false;
  }

  std::vector<VelocityStep> load_velocity_profile_from_csv(const std::string& file_path)
  {
    std::vector<VelocityStep> velocity_steps;
    std::ifstream file(file_path);
    
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", file_path.c_str());
      return velocity_steps;
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
      
      // Skip header line if it contains non-numeric data
      if (!header_skipped) {
        if (line.find("time") != std::string::npos || 
            line.find("vel") != std::string::npos ||
            line.find("waist") != std::string::npos) {
          header_skipped = true;
          RCLCPP_DEBUG(this->get_logger(), "Skipped header line: %s", line.c_str());
          continue;
        }
      }
      
      // Parse CSV line
      std::stringstream ss(line);
      std::string token;
      std::vector<double> values;
      
      while (std::getline(ss, token, ',')) {
        // Trim whitespace
        token.erase(0, token.find_first_not_of(" \t\r\n"));
        token.erase(token.find_last_not_of(" \t\r\n") + 1);
        
        try {
          values.push_back(std::stod(token));
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), 
                       "Failed to parse value '%s' on line %d: %s", 
                       token.c_str(), line_number, e.what());
          return {};
        }
      }
      
      // Expect 4 values: time, waist_vel, shoulder_vel, elbow_vel
      if (values.size() != 4) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Expected 4 values (time, waist_vel, shoulder_vel, elbow_vel) on line %d, got %zu",
                     line_number, values.size());
        return {};
      }
      
      VelocityStep step;
      step.time = values[0];
      step.velocities[0] = values[1];  // waist
      step.velocities[1] = values[2];  // shoulder
      step.velocities[2] = values[3];  // elbow
      
      velocity_steps.push_back(step);
    }
    
    file.close();
    
    if (velocity_steps.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No valid data found in CSV file");
      return {};
    }
    
    RCLCPP_INFO(this->get_logger(), "Loaded %zu velocity steps from CSV", velocity_steps.size());
    return velocity_steps;
  }

  bool validate_velocity_profile(const std::vector<VelocityStep>& velocity_steps)
  {
    if (velocity_steps.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Empty velocity profile");
      return false;
    }

    // Check per-joint velocity limits
    for (size_t i = 0; i < velocity_steps.size(); i++) {
      for (int joint = 0; joint < 3; joint++) {
        if (std::abs(velocity_steps[i].velocities[joint]) > max_velocities_[joint]) {
          RCLCPP_ERROR(this->get_logger(), 
                       "Velocity %.4f rad/s exceeds max %.4f rad/s for %s at t=%.2f", 
                       velocity_steps[i].velocities[joint], 
                       max_velocities_[joint], 
                       joint_names_[joint].c_str(),
                       velocity_steps[i].time);
          return false;
        }
      }

      // Check time ordering
      if (i > 0) {
        if (velocity_steps[i].time <= velocity_steps[i-1].time) {
          RCLCPP_ERROR(this->get_logger(), 
                       "Time steps not in order at index %zu (t=%.2f after t=%.2f)", 
                       i, velocity_steps[i].time, velocity_steps[i-1].time);
          return false;
        }

        // Check per-joint acceleration limits
        double dt = velocity_steps[i].time - velocity_steps[i-1].time;
        for (int joint = 0; joint < 3; joint++) {
          double dv = velocity_steps[i].velocities[joint] - velocity_steps[i-1].velocities[joint];
          double acc = std::abs(dv / dt);
          
          if (acc > max_accelerations_[joint]) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Acceleration %.4f rad/s² exceeds max %.4f rad/s² for %s between t=%.2f and t=%.2f", 
                        acc, max_accelerations_[joint], joint_names_[joint].c_str(),
                        velocity_steps[i-1].time, velocity_steps[i].time);
            return false;
          }
        }
      }
    }

    RCLCPP_INFO(this->get_logger(), "Velocity profile validation passed");
    return true;
  }

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> generate_stepwise_trajectory(
    const std::vector<VelocityStep>& velocity_steps, 
    double dt)
  {
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
    
    if (velocity_steps.empty()) return points;
    
    double total_time = velocity_steps.back().time;
    int num_points = static_cast<int>(std::ceil(total_time / dt)) + 1;
    
    // Initialize positions from current joint states
    std::array<double, 3> positions = current_joint_positions_;
    std::array<double, 3> previous_velocities = velocity_steps[0].velocities;
    std::array<double, 3> current_velocities = velocity_steps[0].velocities;
    size_t step_idx = 0;
    
    for (int i = 0; i < num_points; i++)
    {
      double t = i * dt;
      
      if (t > total_time) {
        t = total_time;
      }
      
      // Save previous velocities
      previous_velocities = current_velocities;
      
      // Update to next velocity step if time has passed
      while (step_idx < velocity_steps.size() - 1 && 
             t >= velocity_steps[step_idx + 1].time)
      {
        step_idx++;
        current_velocities = velocity_steps[step_idx].velocities;
      }
      
      // Create trajectory point
      trajectory_msgs::msg::JointTrajectoryPoint point;
      
      // Set velocities for all 3 joints
      point.velocities = {
        current_velocities[0],
        current_velocities[1],
        current_velocities[2]
      };
      
      // Calculate accelerations for all 3 joints
      point.accelerations = {
        (current_velocities[0] - previous_velocities[0]) / dt,
        (current_velocities[1] - previous_velocities[1]) / dt,
        (current_velocities[2] - previous_velocities[2]) / dt
      };
      
      // Set positions for all 3 joints
      point.positions = {
        positions[0],
        positions[1],
        positions[2]
      };
      
      // Set time
      point.time_from_start = rclcpp::Duration::from_seconds(t);
      
      points.push_back(point);
      
      // Update positions using trapezoidal integration
      for (int j = 0; j < 3; j++) {
        positions[j] += (previous_velocities[j] + current_velocities[j]) * dt / 2.0;
        
        // Position limits check with clamping
        if (enforce_position_limits_) {
          if (positions[j] < min_positions_[j]) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "%s position %.4f rad below minimum %.4f rad at t=%.2fs. Clamping.",
                                 joint_names_[j].c_str(), positions[j], min_positions_[j], t);
            positions[j] = min_positions_[j];
          }
          else if (positions[j] > max_positions_[j]) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "%s position %.4f rad exceeds maximum %.4f rad at t=%.2fs. Clamping.",
                                 joint_names_[j].c_str(), positions[j], max_positions_[j], t);
            positions[j] = max_positions_[j];
          }
        }
      }
      
      if (t >= total_time) break;
    }
    
    return points;
  }

  void log_trajectory_info(
    const FollowJointTrajectory::Goal& goal_msg,
    const std::vector<VelocityStep>& velocity_steps)
  {
    RCLCPP_INFO(this->get_logger(), "=== TRAJECTORY VALIDATION ===");
    RCLCPP_INFO(this->get_logger(), "Joint names: %s, %s, %s", 
                goal_msg.trajectory.joint_names[0].c_str(),
                goal_msg.trajectory.joint_names[1].c_str(),
                goal_msg.trajectory.joint_names[2].c_str());
    RCLCPP_INFO(this->get_logger(), "Total points: %zu", 
                goal_msg.trajectory.points.size());
    RCLCPP_INFO(this->get_logger(), "Sampling rate: %.6f s (%.1f Hz)", 
                sampling_rate_, 1.0/sampling_rate_);
    RCLCPP_INFO(this->get_logger(), "Duration: %.1f seconds", 
                velocity_steps.back().time);

    const auto& first_point = goal_msg.trajectory.points[0];
    const auto& last_point = goal_msg.trajectory.points.back();
    
    double first_time = first_point.time_from_start.sec + 
                        first_point.time_from_start.nanosec / 1e9;
    double last_time = last_point.time_from_start.sec + 
                       last_point.time_from_start.nanosec / 1e9;

    RCLCPP_INFO(this->get_logger(), "First point (t=%.3f s):", first_time);
    RCLCPP_INFO(this->get_logger(), "  Positions: [%.6f, %.6f, %.6f] rad",
                first_point.positions[0], first_point.positions[1], first_point.positions[2]);
    RCLCPP_INFO(this->get_logger(), "  Velocities: [%.6f, %.6f, %.6f] rad/s",
                first_point.velocities[0], first_point.velocities[1], first_point.velocities[2]);
    RCLCPP_INFO(this->get_logger(), "  Accelerations: [%.6f, %.6f, %.6f] rad/s²",
                first_point.accelerations[0], first_point.accelerations[1], first_point.accelerations[2]);

    RCLCPP_INFO(this->get_logger(), "Last point (t=%.3f s):", last_time);
    RCLCPP_INFO(this->get_logger(), "  Positions: [%.6f, %.6f, %.6f] rad",
                last_point.positions[0], last_point.positions[1], last_point.positions[2]);
    RCLCPP_INFO(this->get_logger(), "  Velocities: [%.6f, %.6f, %.6f] rad/s",
                last_point.velocities[0], last_point.velocities[1], last_point.velocities[2]);
    RCLCPP_INFO(this->get_logger(), "  Accelerations: [%.6f, %.6f, %.6f] rad/s²",
                last_point.accelerations[0], last_point.accelerations[1], last_point.accelerations[2]);
    RCLCPP_INFO(this->get_logger(), "===========================");
  }

  void goal_response_callback(const GoalHandleFJT::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      active_goal_handle_ = goal_handle; // Store active goal
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, executing trajectory");
    }
  }

  void feedback_callback(
    GoalHandleFJT::SharedPtr,
    const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
  {
    feedback_counter_++;
    
    if (feedback_counter_ % feedback_throttle_ == 0) {
      if (!feedback->desired.positions.empty() && 
          !feedback->actual.positions.empty() &&
          feedback->desired.positions.size() >= 3 &&
          feedback->actual.positions.size() >= 3) {
        
        RCLCPP_INFO(this->get_logger(), "Feedback:");
        for (size_t i = 0; i < 3; i++) {
          RCLCPP_INFO(this->get_logger(), 
                      "  %s - Desired: %.6f, Actual: %.6f, Error: %.6f",
                      joint_names_[i].c_str(),
                      feedback->desired.positions[i],
                      feedback->actual.positions[i],
                      feedback->error.positions[i]);
        }
      }
    }
  }

  void result_callback(const GoalHandleFJT::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Trajectory succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Trajectory aborted: %s", 
                     result.result->error_string.c_str());
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Trajectory canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }

    if (result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL) {
      RCLCPP_ERROR(this->get_logger(), "Error code: %d - %s",
                   result.result->error_code,
                   result.result->error_string.c_str());
    }
  }
};

}  // namespace stepper_jtc_control

RCLCPP_COMPONENTS_REGISTER_NODE(stepper_jtc_control::MultiJointTrajectoryClient)