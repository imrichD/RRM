#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dzvonar_interface/srv/teach_point.hpp"
#include "block1_Dzvonar/trajectory_io.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>

#include <vector>
#include <string>

class JointLogger : public rclcpp::Node
{
public:
  JointLogger();

private:
  Eigen::Isometry3d dh_transform(double a, double alpha, double d, double theta);
  bool inverse_kinematics(const Eigen::Vector3d &target, std::vector<double> &joint_angles);
  bool forward_kinematics(const std::vector<double> &joint_angles, Eigen::Isometry3d &pose);

  void joint_states_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg);

  void teach_point_callback(
    const std::shared_ptr<dzvonar_interface::srv::TeachPoint::Request> request,
    std::shared_ptr<dzvonar_interface::srv::TeachPoint::Response> response);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Service<dzvonar_interface::srv::TeachPoint>::SharedPtr teach_point_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::vector<double> current_positions_;
  int teach_point_counter_{0};
  const std::string trajectory_file_{"trajectory.csv"};
};