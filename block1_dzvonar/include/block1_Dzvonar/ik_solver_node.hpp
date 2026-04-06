#pragma once

#include "rclcpp/rclcpp.hpp"
#include "block1_dzvonar/srv/ik_all_solutions.hpp"
#include "block1_dzvonar/srv/ik_best_solution.hpp"
#include "block1_dzvonar/srv/ik6_all_solutions.hpp"
#include "block1_dzvonar/srv/ik6_best_solution.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

class IkSolverNode : public rclcpp::Node
{
public:
  IkSolverNode();
  void initialize_from_robot_description();

private:
  struct JointLimit
  {
    double lower;
    double upper;
  };

  struct CandidateSolution
  {
    std::array<double, 3> joints;
  };

  struct CandidateSolution6
  {
    std::array<double, 6> joints;
  };

  void handle_all_solutions(
    const std::shared_ptr<block1_dzvonar::srv::IkAllSolutions::Request> request,
    std::shared_ptr<block1_dzvonar::srv::IkAllSolutions::Response> response);

  void handle_best_solution(
    const std::shared_ptr<block1_dzvonar::srv::IkBestSolution::Request> request,
    std::shared_ptr<block1_dzvonar::srv::IkBestSolution::Response> response);

  void handle_6d_all_solutions(
    const std::shared_ptr<block1_dzvonar::srv::Ik6AllSolutions::Request> request,
    std::shared_ptr<block1_dzvonar::srv::Ik6AllSolutions::Response> response);

  void handle_6d_best_solution(
    const std::shared_ptr<block1_dzvonar::srv::Ik6BestSolution::Request> request,
    std::shared_ptr<block1_dzvonar::srv::Ik6BestSolution::Response> response);

  std::vector<CandidateSolution> compute_solutions(
    double x,
    double y,
    double z,
    const std::array<double, 3> * reference = nullptr) const;

  std::optional<std::array<double, 3>> fit_solution_to_limits(
    const std::array<double, 3> & raw_solution,
    const std::array<double, 3> * reference = nullptr) const;

  std::vector<CandidateSolution6> compute_solutions_6d(
    const geometry_msgs::msg::Pose & target_pose,
    const std::array<double, 6> * reference = nullptr) const;

  std::optional<std::array<double, 6>> fit_solution_to_limits_6d(
    const std::array<double, 6> & raw_solution,
    const std::array<double, 6> * reference = nullptr) const;

  static std::vector<std::array<double, 3>> solve_wrist_zyz(
    const std::array<std::array<double, 3>, 3> & rotation_36);

  static std::vector<JointLimit> parse_joint_limits(const std::string & robot_description);
  static std::optional<std::string> extract_attribute(
    const std::string & block,
    const std::string & attribute_name);
  static bool contains_revolute_type(const std::string & block);
  static double clamp(double value, double lower, double upper);
  static double wrap_to_pi(double angle);
  static double shortest_angular_distance(double from, double to);
  static bool nearly_equal(const std::array<double, 3> & lhs, const std::array<double, 3> & rhs);
  static bool nearly_equal_6d(const std::array<double, 6> & lhs, const std::array<double, 6> & rhs);

  static std::array<std::array<double, 3>, 3> quaternion_to_rotation(
    const geometry_msgs::msg::Pose & pose,
    bool & valid);
  static std::array<std::array<double, 3>, 3> rotation_transpose(
    const std::array<std::array<double, 3>, 3> & matrix);
  static std::array<std::array<double, 3>, 3> rotation_multiply(
    const std::array<std::array<double, 3>, 3> & lhs,
    const std::array<std::array<double, 3>, 3> & rhs);

  std::array<JointLimit, 6> joint_limits_{{
    JointLimit{-3.14159265358979323846, 3.14159265358979323846},
    JointLimit{-3.14159265358979323846, 3.14159265358979323846},
    JointLimit{-3.14159265358979323846, 3.14159265358979323846},
    JointLimit{-3.14159265358979323846, 3.14159265358979323846},
    JointLimit{-3.14159265358979323846, 3.14159265358979323846},
    JointLimit{-3.14159265358979323846, 3.14159265358979323846}}};

  std::size_t loaded_revolute_joint_count_{0};
  rclcpp::Service<block1_dzvonar::srv::IkAllSolutions>::SharedPtr all_solutions_service_;
  rclcpp::Service<block1_dzvonar::srv::IkBestSolution>::SharedPtr best_solution_service_;
  rclcpp::Service<block1_dzvonar::srv::Ik6AllSolutions>::SharedPtr all_solutions_6d_service_;
  rclcpp::Service<block1_dzvonar::srv::Ik6BestSolution>::SharedPtr best_solution_6d_service_;
};
