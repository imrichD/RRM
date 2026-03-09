#include "rclcpp/rclcpp.hpp"
#include "rrm_msgs/msg/command.hpp"
#include <iostream>

class Teleop : public rclcpp::Node
{
public:
  Teleop() : Node("Teleop")
  {
    publisher_ = this->create_publisher<rrm_msgs::msg::Command>(
      "move_command", 10);
    RCLCPP_INFO(this->get_logger(), "Teleop initialized");
  }

  void move(int joint_id, double position)
  {
    rrm_msgs::msg::Command msg;
    msg.joint_id = joint_id;
    msg.position = position;
    publisher_->publish(msg);
  }
  bool move(const std::vector<double>& positions, double max_velocity){
    rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr client_;
  }

private:
  rclcpp::Publisher<rrm_msgs::msg::Command>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  Teleop robot;

  int joint_id = int x = 1;
  double max_velocity = int y = 0;
  robot.move(joint_id, max_velocity);
  for(double i = 1; i<3; i++){
    x = i; 
    y = i / 10;
    }
  rclcpp::shutdown();
  return 0;
}