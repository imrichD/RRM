#include "block1_Dzvonar/logger_node.hpp"
#include "block1_Dzvonar/trajectory_io.hpp"

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>
  
void JointLogger::joint_states_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() < 3) {
    RCLCPP_WARN(this->get_logger(), "Need at least 3 joint positions");
    return;
  }

  current_positions_ = msg->position;

  const double q1 = msg->position[0];
  const double q2 = msg->position[1];
  const double q3 = msg->position[2];

  constexpr double kPi = 3.14159265358979323846;

  auto dh_transform = [](double a, double alpha, double d,
                         double theta) -> Eigen::Matrix4d {
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T << ct, -st * ca, st * sa, a * ct,
      st, ct * ca, -ct * sa, a * st,
      0.0, sa, ca, d,
      0.0, 0.0, 0.0, 1.0;
    return T;
  };

  const double a1 = 0.0;
  const double alpha1 = kPi / 2.0;
  const double d1 = 0.25;
  const double theta1 = q1;

  const double a2 = 0.4;
  const double alpha2 = 0.0;
  const double d2 = 0.0;
  const double theta2 = q2;

  const double a3 = 0.3;
  const double alpha3 = 0.0;
  const double d3 = 0.0;
  const double theta3 = q3;

  const Eigen::Matrix4d T =
    dh_transform(a1, alpha1, d1, theta1) *
    dh_transform(a2, alpha2, d2, theta2) *
    dh_transform(a3, alpha3, d3, theta3);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = "base_link";
  t.child_frame_id = "fk_end_effector";

  t.transform.translation.x = T(0, 3);
  t.transform.translation.y = T(1, 3);
  t.transform.translation.z = T(2, 3);

  const Eigen::Quaterniond q_rot(T.block<3, 3>(0, 0));
  t.transform.rotation.x = q_rot.x();
  t.transform.rotation.y = q_rot.y();
  t.transform.rotation.z = q_rot.z();
  t.transform.rotation.w = q_rot.w();

  tf_broadcaster_->sendTransform(t);
}

JointLogger::JointLogger()
: Node("joint_logger")
{
  subscription_ =
    this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states",
    10,
    std::bind(
      &JointLogger::joint_states_callback,
      this,
      std::placeholders::_1));

  teach_point_service_ =
    this->create_service<dzvonar_interface::srv::TeachPoint>(
    "teach_point",
    std::bind(
      &JointLogger::teach_point_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

void JointLogger::teach_point_callback(
  const std::shared_ptr<dzvonar_interface::srv::TeachPoint::Request> request,
  std::shared_ptr<dzvonar_interface::srv::TeachPoint::Response> response)
{
  if (current_positions_.size() < 3) {
    response->result = false;
    response->message = "No joint state received yet";
    return;
  }

  try {
    save_teach_point(
      teach_point_counter_++,
      current_positions_,
      request->max_velocity,
      trajectory_file_);

    response->result = true;
    response->message =
      "Teach point " + std::to_string(teach_point_counter_ - 1) + " saved";
  } catch (const std::exception & e) {
    response->result = false;
    response->message = std::string("Failed: ") + e.what();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointLogger>());
  rclcpp::shutdown();
  return 0;
}