#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace stepper_jtc_control
{
class StepperTrajectoryClient : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  explicit StepperTrajectoryClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("stepper_trajectory_client", options)
  {
    this->declare_parameter("sampling_rate", 0.02);
    this->declare_parameter("max_velocity", 100.0);
    this->declare_parameter("max_acceleration", 1000.0);
    this->declare_parameter("feedback_throttle", 10);
    
    sampling_rate_ = this->get_parameter("sampling_rate").as_double();
    max_velocity_ = this->get_parameter("max_velocity").as_double();
    max_acceleration_ = this->get_parameter("max_acceleration").as_double();
    feedback_throttle_ = this->get_parameter("feedback_throttle").as_int();
    
    this->client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this,
      "/stepper_trajectory_controller/follow_joint_trajectory");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&StepperTrajectoryClient::send_trajectory, this));
  }

  void send_trajectory()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {"stepper_joint"};

    std::vector<std::pair<double, double>> velocity_steps = {
      {0.0, 5.0},
      {5.0, 25.0},
      {10.0, 10.0},
      {15.0, 0.0},
      {20.0, 50.0},
      {25.0, 0.0},
      {26.0, 0.0}
    };

    if (!validate_velocity_profile(velocity_steps)) {
      RCLCPP_ERROR(this->get_logger(), "Velocity profile validation failed!");
      return;
    }

    goal_msg.trajectory.points = generate_stepwise_trajectory(velocity_steps, sampling_rate_);

    goal_msg.trajectory.header.stamp.sec = 0;
    goal_msg.trajectory.header.stamp.nanosec = 0;
    goal_msg.trajectory.header.frame_id = "";

    RCLCPP_INFO(this->get_logger(), "Sending trajectory goal with %zu points over %.1f seconds", 
                goal_msg.trajectory.points.size(),
                velocity_steps.back().first);

    RCLCPP_INFO(this->get_logger(), "=== TRAJECTORY VALIDATION ===");
    RCLCPP_INFO(this->get_logger(), "Joint names: %s", goal_msg.trajectory.joint_names[0].c_str());
    RCLCPP_INFO(this->get_logger(), "Total points: %zu", goal_msg.trajectory.points.size());
    RCLCPP_INFO(this->get_logger(), "Sampling rate: %.6f s (%.1f Hz)", sampling_rate_, 1.0/sampling_rate_);

    double first_time = goal_msg.trajectory.points[0].time_from_start.sec + 
                        goal_msg.trajectory.points[0].time_from_start.nanosec / 1e9;
    double last_time = goal_msg.trajectory.points.back().time_from_start.sec + 
                       goal_msg.trajectory.points.back().time_from_start.nanosec / 1e9;

    RCLCPP_INFO(this->get_logger(), "First point - pos: %.6f, vel: %.6f, acc: %.6f, time: %.9f s", 
                goal_msg.trajectory.points[0].positions[0],
                goal_msg.trajectory.points[0].velocities[0],
                goal_msg.trajectory.points[0].accelerations[0],
                first_time);
    RCLCPP_INFO(this->get_logger(), "Last point - pos: %.6f, vel: %.6f, acc: %.6f, time: %.9f s", 
                goal_msg.trajectory.points.back().positions[0],
                goal_msg.trajectory.points.back().velocities[0],
                goal_msg.trajectory.points.back().accelerations[0],
                last_time);
    RCLCPP_INFO(this->get_logger(), "===========================");

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&StepperTrajectoryClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&StepperTrajectoryClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&StepperTrajectoryClient::result_callback, this, _1);

    feedback_counter_ = 0;
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void send_velocity_profile(const std::vector<double>& velocities, double dt = 0.02)
  {
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {"stepper_joint"};
    goal_msg.trajectory.header.stamp.sec = 0;
    goal_msg.trajectory.header.stamp.nanosec = 0;

    double position = 0.0;
    double prev_velocity = 0.0;
    
    for (size_t i = 0; i < velocities.size(); i++)
    {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.velocities = {velocities[i]};
      
      double acceleration = (velocities[i] - prev_velocity) / dt;
      point.accelerations = {acceleration};
      
      point.positions = {position};
      point.time_from_start = rclcpp::Duration::from_seconds(i * dt);
      
      goal_msg.trajectory.points.push_back(point);
      
      position += (prev_velocity + velocities[i]) * dt / 2.0;
      prev_velocity = velocities[i];
    }

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&StepperTrajectoryClient::result_callback, this, std::placeholders::_1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "Sent velocity profile with %zu points", velocities.size());
  }

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  double sampling_rate_;
  double max_velocity_;
  double max_acceleration_;
  int feedback_throttle_;
  int feedback_counter_;

  bool validate_velocity_profile(const std::vector<std::pair<double, double>>& velocity_steps)
  {
    if (velocity_steps.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Empty velocity profile");
      return false;
    }

    for (size_t i = 0; i < velocity_steps.size(); i++) {
      if (std::abs(velocity_steps[i].second) > max_velocity_) {
        RCLCPP_ERROR(this->get_logger(), "Velocity %.2f exceeds max %.2f at t=%.2f", 
                     velocity_steps[i].second, max_velocity_, velocity_steps[i].first);
        return false;
      }

      if (i > 0) {
        if (velocity_steps[i].first <= velocity_steps[i-1].first) {
          RCLCPP_ERROR(this->get_logger(), "Time steps not in order at index %zu", i);
          return false;
        }

        double dt = velocity_steps[i].first - velocity_steps[i-1].first;
        double dv = velocity_steps[i].second - velocity_steps[i-1].second;
        double acc = std::abs(dv / dt);
        
        if (acc > max_acceleration_) {
          RCLCPP_WARN(this->get_logger(), "Acceleration %.2f exceeds max %.2f between t=%.2f and t=%.2f", 
                      acc, max_acceleration_, velocity_steps[i-1].first, velocity_steps[i].first);
        }
      }
    }

    return true;
  }

  void goal_response_callback(const GoalHandleFJT::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
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
          !feedback->actual.positions.empty()) {
        RCLCPP_INFO(this->get_logger(), "Feedback - Desired pos: %.6f, Actual pos: %.6f, Error: %.6f",
                    feedback->desired.positions[0],
                    feedback->actual.positions[0],
                    feedback->error.positions[0]);
      }
    }
  }

  void result_callback(const GoalHandleFJT::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Trajectory succeeded!");
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

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> generate_stepwise_trajectory(
    const std::vector<std::pair<double, double>>& velocity_steps, 
    double dt)
  {
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
    
    if (velocity_steps.empty()) return points;
    
    double total_time = velocity_steps.back().first;
    int num_points = static_cast<int>(std::ceil(total_time / dt)) + 1;
    
    double position = 0.0;
    double previous_velocity = velocity_steps[0].second;
    double current_velocity = velocity_steps[0].second;
    size_t step_idx = 0;
    
    for (int i = 0; i < num_points; i++)
    {
      double t = i * dt;
      
      if (t > total_time) {
        t = total_time;
      }
      
      previous_velocity = current_velocity;
      
      while (step_idx < velocity_steps.size() - 1 && 
             t >= velocity_steps[step_idx + 1].first)
      {
        step_idx++;
        current_velocity = velocity_steps[step_idx].second;
      }
      
      trajectory_msgs::msg::JointTrajectoryPoint point;
      
      point.velocities = {current_velocity};
      
      double acceleration = (current_velocity - previous_velocity) / dt;
      point.accelerations = {acceleration};
      
      point.positions = {position};
      
      point.time_from_start = rclcpp::Duration::from_seconds(t);
      
      points.push_back(point);
      
      position += (previous_velocity + current_velocity) * dt / 2.0;
      
      if (t >= total_time) break;
    }
    
    return points;
  }

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> generate_velocity_segments(
    const std::vector<double>& velocities,
    const std::vector<double>& durations,
    double dt = 0.02)
  {
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
    
    if (velocities.size() != durations.size()) {
      RCLCPP_ERROR(this->get_logger(), "Velocities and durations size mismatch!");
      return points;
    }
    
    double position = 0.0;
    double time_elapsed = 0.0;
    
    for (size_t seg = 0; seg < velocities.size(); seg++)
    {
      double velocity = velocities[seg];
      double duration = durations[seg];
      int n_points = static_cast<int>(duration / dt);
      
      for (int i = 0; i < n_points; i++)
      {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.velocities = {velocity};
        point.positions = {position};
        point.time_from_start = rclcpp::Duration::from_seconds(time_elapsed);
        
        points.push_back(point);
        
        position += velocity * dt;
        time_elapsed += dt;
      }
    }
    
    return points;
  }
};

}  // namespace stepper_jtc_control

RCLCPP_COMPONENTS_REGISTER_NODE(stepper_jtc_control::StepperTrajectoryClient)
