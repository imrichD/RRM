#include "block1_Dzvonar/motion_manager_node.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <cmath>
#include <future>
#include <limits>
#include <stdexcept>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDefaultTimeoutSeconds = 15.0;
}  // namespace

MotionManagerNode::MotionManagerNode()
: Node("motion_manager")
{
  this->declare_parameter<double>("max_velocity", max_velocity_);
  this->get_parameter("max_velocity", max_velocity_);

  client_node_ = std::make_shared<rclcpp::Node>("motion_manager_clients");
  ik_client_ = client_node_->create_client<block1_dzvonar::srv::IkBestSolution>("ik_best_solution");
  ik6_client_ = client_node_->create_client<block1_dzvonar::srv::Ik6BestSolution>("ik6_best_solution");
  move_client_ = client_node_->create_client<rrm_msgs::srv::Command>("move_command");

  joint_states_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states",
    10,
    std::bind(&MotionManagerNode::joint_states_callback, this, std::placeholders::_1));

  move_cartesian_service_ = this->create_service<block1_dzvonar::srv::MoveCartesian>(
    "move_cartesian",
    std::bind(
      &MotionManagerNode::move_cartesian_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  move_pose_service_ = this->create_service<block1_dzvonar::srv::MovePose>(
    "move_pose",
    std::bind(
      &MotionManagerNode::move_pose_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

void MotionManagerNode::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(current_positions_mutex_);
  current_positions_ = msg->position;
}

std::vector<double> MotionManagerNode::current_joint_positions_snapshot() const
{
  std::lock_guard<std::mutex> lock(current_positions_mutex_);
  return current_positions_;
}

double MotionManagerNode::shortest_angular_distance(double from, double to)
{
  return std::remainder(to - from, kTwoPi);
}

bool MotionManagerNode::get_best_ik_solution(
  const std::array<double, 3> & target,
  std::array<double, 3> & result,
  std::string & message)
{
  if (!ik_client_->wait_for_service(std::chrono::seconds(1))) {
    message = "IK solver service is not available";
    return false;
  }

  auto request = std::make_shared<block1_dzvonar::srv::IkBestSolution::Request>();
  request->x = target[0];
  request->y = target[1];
  request->z = target[2];

  const auto current_positions = current_joint_positions_snapshot();
  if (current_positions.size() < 3) {
    message = "No joint state received yet";
    return false;
  }

  request->current_j1 = current_positions[0];
  request->current_j2 = current_positions[1];
  request->current_j3 = current_positions[2];

  auto future = ik_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
      client_node_->get_node_base_interface(), future,
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(kDefaultTimeoutSeconds))) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    message = "Timed out while waiting for the IK solver";
    return false;
  }

  auto response = future.get();
  if (!response->success) {
    message = response->message;
    return false;
  }

  result[0] = response->joints[0];
  result[1] = response->joints[1];
  result[2] = response->joints[2];
  message = response->message;
  return true;
}

bool MotionManagerNode::get_best_ik_solution_6d(
  const geometry_msgs::msg::Pose & target_pose,
  std::array<double, 6> & result,
  std::string & message)
{
  if (!ik6_client_->wait_for_service(std::chrono::seconds(1))) {
    message = "6DOF IK solver service is not available";
    return false;
  }

  auto request = std::make_shared<block1_dzvonar::srv::Ik6BestSolution::Request>();
  request->target_pose = target_pose;

  const auto current_positions = current_joint_positions_snapshot();
  if (current_positions.size() < 6) {
    message = "No 6-joint state received yet";
    return false;
  }

  for (std::size_t i = 0; i < result.size(); ++i) {
    request->current_joints[i] = current_positions[i];
  }

  auto future = ik6_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
      client_node_->get_node_base_interface(), future,
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(kDefaultTimeoutSeconds))) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    message = "Timed out while waiting for the 6DOF IK solver";
    return false;
  }

  auto response = future.get();
  if (!response->success) {
    message = response->message;
    return false;
  }

  for (std::size_t i = 0; i < result.size(); ++i) {
    result[i] = response->joints[i];
  }
  message = response->message;
  return true;
}

bool MotionManagerNode::execute_move_command(
  const std::vector<double> & target_joints,
  std::string & message)
{
  if (!move_client_->wait_for_service(std::chrono::seconds(1))) {
    message = "move_command service is not available";
    return false;
  }

  const auto current_positions = current_joint_positions_snapshot();
  if (current_positions.size() < target_joints.size()) {
    message = "No complete joint state received yet";
    return false;
  }

  auto request = std::make_shared<rrm_msgs::srv::Command::Request>();
  request->positions = target_joints;
  request->velocities.resize(target_joints.size(), 0.0);

  double max_delta = 0.0;
  std::vector<double> deltas(target_joints.size(), 0.0);
  for (std::size_t index = 0; index < deltas.size(); ++index) {
    deltas[index] = shortest_angular_distance(current_positions[index], target_joints[index]);
    max_delta = std::max(max_delta, std::abs(deltas[index]));
  }

  if (max_delta < 1e-9) {
    message = "Target configuration already reached";
    return true;
  }

  for (std::size_t index = 0; index < deltas.size(); ++index) {
    request->velocities[index] = max_velocity_ * std::abs(deltas[index]) / max_delta;
  }

  auto future = move_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
      client_node_->get_node_base_interface(), future,
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(kDefaultTimeoutSeconds))) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    message = "Timed out while waiting for move_command response";
    return false;
  }

  auto response = future.get();
  message = response->message;
  return response->result_code == 0;
}

void MotionManagerNode::move_pose_callback(
  const std::shared_ptr<block1_dzvonar::srv::MovePose::Request> request,
  std::shared_ptr<block1_dzvonar::srv::MovePose::Response> response)
{
  std::array<double, 6> target_joints{};
  std::string message;

  if (!get_best_ik_solution_6d(request->target_pose, target_joints, message)) {
    response->success = false;
    response->message = message;
    return;
  }

  if (!execute_move_command(std::vector<double>(target_joints.begin(), target_joints.end()), message)) {
    response->success = false;
    response->message = message;
    return;
  }

  response->success = true;
  response->message = "Pose move completed: " + message;
}

void MotionManagerNode::move_cartesian_callback(
  const std::shared_ptr<block1_dzvonar::srv::MoveCartesian::Request> request,
  std::shared_ptr<block1_dzvonar::srv::MoveCartesian::Response> response)
{
  const std::array<double, 3> target{{request->x, request->y, request->z}};
  std::array<double, 3> target_joints{};
  std::string message;

  if (!get_best_ik_solution(target, target_joints, message)) {
    response->success = false;
    response->message = message;
    return;
  }

  const auto current_positions = current_joint_positions_snapshot();
  std::vector<double> full_command(current_positions.size(), 0.0);
  for (std::size_t i = 0; i < full_command.size(); ++i) {
    full_command[i] = i < 3 ? target_joints[i] : current_positions[i];
  }

  if (!execute_move_command(full_command, message)) {
    response->success = false;
    response->message = message;
    return;
  }

  response->success = true;
  response->message = "Cartesian move completed: " + message;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
