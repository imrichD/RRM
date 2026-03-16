#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rrm_msgs/srv/command.hpp"
#include <vector>

class Teleop : public rclcpp::Node
{
public:
  Teleop();
  bool move(const std::vector<double>& positions, double max_velocity);

private:
  rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr client_;
};