#include "block1_Dzvonar/logger_node.hpp"
#include "block1_Dzvonar/trajectory_io.hpp"

JointLogger::JointLogger() : Node("joint_logger")
{
  subscription_ =
    this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&JointLogger::joint_states_callback, this,
        std::placeholders::_1));

  teach_point_service_ =
    this->create_service<dzvonar_interface::srv::TeachPoint>(
      "teach_point",
      std::bind(&JointLogger::teach_point_callback, this,
        std::placeholders::_1, std::placeholders::_2));
}

void JointLogger::joint_states_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  current_positions_ = msg->position;
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
      teach_point_counter_++, current_positions_,
      request->max_velocity, trajectory_file_);
    response->result = true;
    response->message =
      "Teach point " + std::to_string(teach_point_counter_ - 1) + " saved";
  } catch (const std::exception& e) {
    response->result = false;
    response->message = std::string("Failed: ") + e.what();
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointLogger>());
  rclcpp::shutdown();
  return 0;
}