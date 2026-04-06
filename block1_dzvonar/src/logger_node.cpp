#include "block1_Dzvonar/logger_node.hpp"
#include <memory>

#include <tf2/exceptions.h>

void JointLogger::joint_states_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() < 3) {
    RCLCPP_WARN(this->get_logger(), "Need at least 3 joint positions");
    return;
  }

  current_positions_ = msg->position;

  try {
    const auto tool_tf = tf_buffer_->lookupTransform("base_link", "tool0", tf2::TimePointZero);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "fk_end_effector";
    t.transform = tool_tf.transform;

    tf_broadcaster_->sendTransform(t);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Cannot read TF base_link -> tool0 yet: %s",
      ex.what());
  }
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

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointLogger>());
  rclcpp::shutdown();
  return 0;
}
