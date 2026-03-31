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
  if (msg->position.size() < 3) return;
  this->current_positions_ = msg->position;

  double q1 = msg->position[0];
  double q2 = msg->position[1];
  double q3 = msg->position[2];

  // --- A1: Rotácia okolo Z (Joint 1) ---
  Eigen::Matrix4d A1 = Eigen::Matrix4d::Identity();
  A1 << cos(q1), -sin(q1), 0, 0,
        sin(q1),  cos(q1), 0, 0,
        0,        0,       1, 0,
        0,        0,       0, 1;

  // --- A2: Rotácia okolo Y (Joint 2) ---
  Eigen::Matrix4d A2 = Eigen::Matrix4d::Identity();
  A2 << cos(q2),  0, sin(q2), 0,
        0,        1, 0,       0,
        -sin(q2), 0, cos(q2), 0,
        0,        0, 0,       1;

  // --- A3: Posun o 0.203 a rotácia okolo Y (Joint 3) ---
  Eigen::Matrix4d A3 = Eigen::Matrix4d::Identity();
  double d_arm = 0.203; // Dĺžka ramena z URDF
  A3 << cos(q3),  0, sin(q3), 0,
        0,        1, 0,       0,
        -sin(q3), 0, cos(q3), d_arm,
        0,        0, 0,       1;

  // --- A_tool: Posun na úplný koniec (Tool0) ---
  Eigen::Matrix4d A_tool = Eigen::Matrix4d::Identity();
  A_tool(2, 3) = 0.203; // Posledný kus ramena k tool0

  // Celkový výpočet: T = A1 * A2 * A3 * A_tool
  Eigen::Matrix4d T = A1 * A2 * A3 * A_tool;

  // --- Odoslanie do TF ---
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = "base_link";
  t.child_frame_id = "tool0_calculated";
  
  t.transform.translation.x = T(0, 3);
  t.transform.translation.y = T(1, 3);
  t.transform.translation.z = T(2, 3);
  
  Eigen::Quaterniond q_rot(T.block<3, 3>(0, 0));
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