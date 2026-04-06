#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rrm_msgs/srv/command.hpp"
#include "dzvonar_interface/srv/teach_point.hpp"

#include <vector>

class Teleop : public rclcpp::Node
{
public:
  Teleop();
  bool move(const std::vector<double>& positions, double max_velocity);
  bool save_current_point(double max_velocity);
  void run_trajectory();
  void run_menu();

private:
  rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr move_client_;
  rclcpp::Client<dzvonar_interface::srv::TeachPoint>::SharedPtr teach_client_;
};