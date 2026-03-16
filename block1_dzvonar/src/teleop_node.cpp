#include "block1_Dzvonar/teleop_node.hpp"
#include <algorithm>
#include <cmath>

Teleop::Teleop() : Node("teleop")
{
  client_ = this->create_client<rrm_msgs::srv::Command>("move_command");
}

bool Teleop::move(const std::vector<double>& positions, double max_velocity)
{
  // Wait for service to be available
  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for service. Exiting.");
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(), "Service not available, waiting...");
  }

  
  double max_disp = 0.0;
  for (const auto& p : positions) {
    max_disp = std::max(max_disp, std::abs(p));
  }

  std::vector<double> velocities(positions.size(), 0.0);
  if (max_disp > 0.0) {
    for (size_t i = 0; i < positions.size(); i++) {
      velocities[i] = max_velocity * std::abs(positions[i]) / max_disp;
    }
  }

  // Build and send request
  auto request = std::make_shared<rrm_msgs::srv::Command::Request>();
  request->positions = positions;
  request->velocities = velocities;

  auto result = client_->async_send_request(request);

  // Wait for response with 5s timeout
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Service call failed");
    return false;
  }

  auto response = result.get();
  RCLCPP_INFO(this->get_logger(), "Service call succeeded: result_code = %d, message = %s",
    response->result_code, response->message.c_str());

  return response->result_code == 0;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto robot = std::make_shared<Teleop>();


  std::vector<double> positions = {0, -1, 2.5};
  double max_velocity = 0.5;

  robot->move(positions, max_velocity);
  
  rclcpp::shutdown();
  return 0; 
}