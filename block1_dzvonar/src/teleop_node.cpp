#include "block1_Dzvonar/teleop_node.hpp"
#include "block1_Dzvonar/trajectory_io.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

Teleop::Teleop() : Node("teleop")
{
  move_client_ =
    this->create_client<rrm_msgs::srv::Command>("move_command");
  teach_client_ =
    this->create_client<dzvonar_interface::srv::TeachPoint>("teach_point");
}

bool Teleop::move(
  const std::vector<double>& positions, double max_velocity)
{
  while (!move_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) return false;
    RCLCPP_INFO(this->get_logger(), "Waiting for move_command service...");
  }

  double max_disp = 0.0;
  for (const auto& p : positions) {
    max_disp = std::max(max_disp, std::abs(p));
  }

  auto request = std::make_shared<rrm_msgs::srv::Command::Request>();
  request->positions = positions;
  request->velocities.resize(positions.size(), 0.0);
  if (max_disp > 0.0) {
    for (size_t i = 0; i < positions.size(); i++) {
      request->velocities[i] =
        max_velocity * std::abs(positions[i]) / max_disp;
    }
  }

  auto future = move_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), future,
        std::chrono::seconds(15)) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Move failed");
    return false;
  }

  auto result = future.get();
RCLCPP_INFO(this->get_logger(), "result_code=%d message=%s",
  result->result_code, result->message.c_str());
return result->result_code == 0;
}

bool Teleop::save_current_point(double max_velocity)
{
  while (!teach_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) return false;
    RCLCPP_INFO(this->get_logger(), "Waiting for teach_point service...");
  }

  auto request = std::make_shared<dzvonar_interface::srv::TeachPoint::Request>();
  request->max_velocity = max_velocity;

  auto future = teach_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete( 
    this->get_node_base_interface(), future, std::chrono::seconds(15)) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Save teach point failed");
    return false;
  }

  auto response = future.get();
  std::cout << (response->result ? "[OK] " : "[FAIL] ")
            << response->message << "\n";
  return response->result;
}

void Teleop::run_trajectory()
{
  std::vector<TeachPoint> points;
  try {
    points = load_teach_points("trajectory.csv");
  } catch (const std::exception& e) {
    std::cout << "[FAIL] " << e.what() << "\n";
    return;
  }

  if (points.empty()) {
    std::cout << "[FAIL] No teach points in file\n";
    return;
  }

  std::cout << "Running " << points.size() << " teach points...\n";
  for (const auto& pt : points) {
    std::cout << "  -> Point " << pt.id << "\n";
    move(pt.positions, pt.max_velocity);
  }
  std::cout << "[OK] Trajectory done\n";
}

void Teleop::run_menu()
{
  char choice;
  while (rclcpp::ok()) {
    std::cout << "\n=== Robot Control ===\n";
    std::cout << "[m] Move robot\n";
    std::cout << "[s] Save teach point\n";
    std::cout << "[r] Run trajectory\n";
    std::cout << "[q] Quit\n";
    std::cout << "> ";
    std::cin >> choice;

    // clear bad input state
    if (std::cin.fail()) {
      std::cin.clear();
      std::cin.ignore(1000, '\n');
      continue;
    }

    if (choice == 'm') {
      double j1, j2, j3, vel;
      std::cout << "Positions j1 j2 j3 (napr: 1.0 -0.5 2.5): ";
      if (!(std::cin >> j1 >> j2 >> j3)) {
        std::cout << "[FAIL] Zly format, pouzij medzery medzi hodnotami\n";
        std::cin.clear();
        std::cin.ignore(1000, '\n');
        continue;
      }
      std::cout << "Max velocity (napr: 0.5): ";
      if (!(std::cin >> vel)) {
        std::cout << "[FAIL] Zla hodnota velocity\n";
        std::cin.clear();
        std::cin.ignore(1000, '\n');
        continue;
      }
      bool ok = move({j1, j2, j3}, vel);
      std::cout << (ok ? "[OK] Move done\n" : "[FAIL] Move failed\n");

    } else if (choice == 's') {
      double vel;
      std::cout << "Max velocity (napr: 0.5): ";
      if (!(std::cin >> vel)) {
        std::cout << "[FAIL] Zla hodnota velocity\n";
        std::cin.clear();
        std::cin.ignore(1000, '\n');
        continue;
      }
      save_current_point(vel);

    } else if (choice == 'r') {
      run_trajectory();

    } else if (choice == 'q') {
      break;
    }
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto robot = std::make_shared<Teleop>();
  robot->run_menu();
  rclcpp::shutdown();
  return 0;
}