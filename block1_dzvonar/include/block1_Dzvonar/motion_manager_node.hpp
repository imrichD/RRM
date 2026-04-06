#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rrm_msgs/srv/command.hpp"
#include "block1_dzvonar/srv/ik_best_solution.hpp"
#include "block1_dzvonar/srv/ik6_best_solution.hpp"
#include "block1_dzvonar/srv/move_cartesian.hpp"
#include "block1_dzvonar/srv/move_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class MotionManagerNode : public rclcpp::Node
{
public:
  MotionManagerNode();

private:
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void move_cartesian_callback(
    const std::shared_ptr<block1_dzvonar::srv::MoveCartesian::Request> request,
    std::shared_ptr<block1_dzvonar::srv::MoveCartesian::Response> response);

  void move_pose_callback(
    const std::shared_ptr<block1_dzvonar::srv::MovePose::Request> request,
    std::shared_ptr<block1_dzvonar::srv::MovePose::Response> response);

  bool get_best_ik_solution(
    const std::array<double, 3> & target,
    std::array<double, 3> & result,
    std::string & message);

  bool get_best_ik_solution_6d(
    const geometry_msgs::msg::Pose & target_pose,
    std::array<double, 6> & result,
    std::string & message);

  bool execute_move_command(
    const std::vector<double> & target_joints,
    std::string & message);

  std::vector<double> current_joint_positions_snapshot() const;
  static double shortest_angular_distance(double from, double to);

  mutable std::mutex current_positions_mutex_;
  std::vector<double> current_positions_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
  rclcpp::Service<block1_dzvonar::srv::MoveCartesian>::SharedPtr move_cartesian_service_;
  rclcpp::Service<block1_dzvonar::srv::MovePose>::SharedPtr move_pose_service_;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<block1_dzvonar::srv::IkBestSolution>::SharedPtr ik_client_;
  rclcpp::Client<block1_dzvonar::srv::Ik6BestSolution>::SharedPtr ik6_client_;
  rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr move_client_;

  double max_velocity_{0.5};
};
