#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "block1_dzvonar/srv/move_cartesian.hpp"
#include "block1_dzvonar/srv/move_pose.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <thread>

class CartesianControlClient
{
public:
  CartesianControlClient()
  : node_(std::make_shared<rclcpp::Node>("cartesian_control_client"))
  {
    client_ = node_->create_client<block1_dzvonar::srv::MoveCartesian>("move_cartesian");
    pose_client_ = node_->create_client<block1_dzvonar::srv::MovePose>("move_pose");
  }

  std::shared_ptr<rclcpp::Node> node_;

  void run()
  {
    std::cout << "\n========================================\n";
    std::cout << "   Cartesian Control Interface (RRM)     \n";
    std::cout << "========================================\n\n";

    while (rclcpp::ok()) {
      std::cout << "\nOptions:\n";
      std::cout << "  [m] Move to Cartesian position\n";
      std::cout << "  [p] Move to Pose (position + quaternion)\n";
      std::cout << "  [q] Quit\n";
      std::cout << "> ";

      char choice;
      std::cin >> choice;

      if (std::cin.fail()) {
        std::cin.clear();
        std::cin.ignore(1000, '\n');
        std::cout << "[ERROR] Invalid input\n";
        continue;
      }

      if (choice == 'm' || choice == 'M') {
        move_cartesian();
      } else if (choice == 'p' || choice == 'P') {
        move_pose();
      } else if (choice == 'q' || choice == 'Q') {
        std::cout << "[INFO] Exiting...\n";
        break;
      } else {
        std::cout << "[ERROR] Unknown option\n";
      }
    }
  }

private:
  void move_cartesian()
  {
    double x, y, z;
    std::cout << "\nEnter target Cartesian position:\n";
    std::cout << "  X (meters): ";
    if (!(std::cin >> x)) {
      std::cin.clear();
      std::cin.ignore(1000, '\n');
      std::cout << "[ERROR] Invalid X coordinate\n";
      return;
    }
    std::cout << "  Y (meters): ";
    if (!(std::cin >> y)) {
      std::cin.clear();
      std::cin.ignore(1000, '\n');
      std::cout << "[ERROR] Invalid Y coordinate\n";
      return;
    }
    std::cout << "  Z (meters): ";
    if (!(std::cin >> z)) {
      std::cin.clear();
      std::cin.ignore(1000, '\n');
      std::cout << "[ERROR] Invalid Z coordinate\n";
      return;
    }

    std::cout << "\n[INFO] Moving to (" << x << ", " << y << ", " << z << ")...\n";

    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      std::cout << "[ERROR] move_cartesian service not available\n";
      return;
    }

    auto request = std::make_shared<block1_dzvonar::srv::MoveCartesian::Request>();
    request->x = x;
    request->y = y;
    request->z = z;

    auto result = client_->async_send_request(
      request,
      [this](rclcpp::Client<block1_dzvonar::srv::MoveCartesian>::SharedFuture future) {
        const auto response = future.get();
        if (response->success) {
          std::cout << "[OK] " << response->message << "\n";
        } else {
          std::cout << "[FAIL] " << response->message << "\n";
        }
      });
  }

  void move_pose()
  {
    double x, y, z, qx, qy, qz, qw;
    std::cout << "\nEnter target Pose:\n";
    std::cout << "  X (meters): ";
    if (!(std::cin >> x)) { std::cin.clear(); std::cin.ignore(1000, '\n'); std::cout << "[ERROR] Invalid X coordinate\n"; return; }
    std::cout << "  Y (meters): ";
    if (!(std::cin >> y)) { std::cin.clear(); std::cin.ignore(1000, '\n'); std::cout << "[ERROR] Invalid Y coordinate\n"; return; }
    std::cout << "  Z (meters): ";
    if (!(std::cin >> z)) { std::cin.clear(); std::cin.ignore(1000, '\n'); std::cout << "[ERROR] Invalid Z coordinate\n"; return; }
    std::cout << "  Qx: ";
    if (!(std::cin >> qx)) { std::cin.clear(); std::cin.ignore(1000, '\n'); std::cout << "[ERROR] Invalid Qx\n"; return; }
    std::cout << "  Qy: ";
    if (!(std::cin >> qy)) { std::cin.clear(); std::cin.ignore(1000, '\n'); std::cout << "[ERROR] Invalid Qy\n"; return; }
    std::cout << "  Qz: ";
    if (!(std::cin >> qz)) { std::cin.clear(); std::cin.ignore(1000, '\n'); std::cout << "[ERROR] Invalid Qz\n"; return; }
    std::cout << "  Qw: ";
    if (!(std::cin >> qw)) { std::cin.clear(); std::cin.ignore(1000, '\n'); std::cout << "[ERROR] Invalid Qw\n"; return; }

    if (!pose_client_->wait_for_service(std::chrono::seconds(1))) {
      std::cout << "[ERROR] move_pose service not available\n";
      return;
    }

    auto request = std::make_shared<block1_dzvonar::srv::MovePose::Request>();
    request->target_pose.position.x = x;
    request->target_pose.position.y = y;
    request->target_pose.position.z = z;
    request->target_pose.orientation.x = qx;
    request->target_pose.orientation.y = qy;
    request->target_pose.orientation.z = qz;
    request->target_pose.orientation.w = qw;

    std::cout << "\n[INFO] Moving to pose...\n";
    auto result = pose_client_->async_send_request(
      request,
      [this](rclcpp::Client<block1_dzvonar::srv::MovePose>::SharedFuture future) {
        const auto response = future.get();
        if (response->success) {
          std::cout << "[OK] " << response->message << "\n";
        } else {
          std::cout << "[FAIL] " << response->message << "\n";
        }
      });
  }

  rclcpp::Client<block1_dzvonar::srv::MoveCartesian>::SharedPtr client_;
  rclcpp::Client<block1_dzvonar::srv::MovePose>::SharedPtr pose_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto client = std::make_shared<CartesianControlClient>();
  
  std::thread spin_thread([&client]() {
    rclcpp::spin(client->node_);
  });

  client->run();

  rclcpp::shutdown();
  spin_thread.join();

  return 0;
}
