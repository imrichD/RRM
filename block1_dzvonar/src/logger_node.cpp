#include "block1_Dzvonar/teleop_node.hpp"

JointLogger::JointLogger() : Node("joint_logger")
{
  subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&JointLogger::joint_states_callback, this,
      std::placeholders::_1));
}

void JointLogger::joint_states_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); i++) {
    RCLCPP_INFO(this->get_logger(), "Joint: %s, Position: %f",
      msg->name[i].c_str(), msg->position[i]);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto logger = std::make_shared<JointLogger>();
  rclcpp::spin(logger);
  rclcpp::shutdown();
  return 0;
}