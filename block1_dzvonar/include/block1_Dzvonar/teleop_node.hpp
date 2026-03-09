#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rrm_msgs/srv/command.hpp"

class JointLogger : public rclcpp::Node
{
public:
  JointLogger();
  
  bool move(const std::vector<double>& positions, double max_velocity);
  rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr client_;

private:
  void joint_states_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

};