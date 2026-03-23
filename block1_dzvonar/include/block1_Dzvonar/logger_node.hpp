#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dzvonar_interface/srv/teach_point.hpp"
#include "block1_Dzvonar/trajectory_io.hpp"

#include <vector>
#include <string>

class JointLogger : public rclcpp::Node
{
public:
  JointLogger();

private:
  void joint_states_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg);

  void teach_point_callback(
    const std::shared_ptr<dzvonar_interface::srv::TeachPoint::Request> request,
    std::shared_ptr<dzvonar_interface::srv::TeachPoint::Response> response);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Service<dzvonar_interface::srv::TeachPoint>::SharedPtr teach_point_service_;

  std::vector<double> current_positions_;
  int teach_point_counter_{0};
  const std::string trajectory_file_{"trajectory.csv"};
};