#include "block1_Dzvonar/ik_solver_node.hpp"

#include <rclcpp/parameter_client.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kLink1Offset = 0.0;
constexpr double kLink2Length = 0.203;
constexpr double kLink3Length = 0.203;
constexpr double kArm6Link2ToJoint3 = 0.203;
constexpr double kArm6Joint3ToWrist = 0.203;
constexpr double kArm6ToolOffset = 0.12;
constexpr double kTolerance = 1e-6;

std::string trim_copy(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\n\r");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\n\r");
  return value.substr(first, last - first + 1);
}
}  // namespace

IkSolverNode::IkSolverNode()
: Node("ik_solver")
{
  all_solutions_service_ = this->create_service<block1_dzvonar::srv::IkAllSolutions>(
    "ik_all_solutions",
    std::bind(
      &IkSolverNode::handle_all_solutions,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  best_solution_service_ = this->create_service<block1_dzvonar::srv::IkBestSolution>(
    "ik_best_solution",
    std::bind(
      &IkSolverNode::handle_best_solution,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  all_solutions_6d_service_ = this->create_service<block1_dzvonar::srv::Ik6AllSolutions>(
    "ik6_all_solutions",
    std::bind(
      &IkSolverNode::handle_6d_all_solutions,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  best_solution_6d_service_ = this->create_service<block1_dzvonar::srv::Ik6BestSolution>(
    "ik6_best_solution",
    std::bind(
      &IkSolverNode::handle_6d_best_solution,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

void IkSolverNode::initialize_from_robot_description()
{
  const auto remote_node = this->declare_parameter<std::string>(
    "robot_description_node", "/robot_state_publisher");
  const auto remote_parameter = this->declare_parameter<std::string>(
    "robot_description_parameter", "robot_description");

  auto client = std::make_shared<rclcpp::SyncParametersClient>(
    this->shared_from_this(),
    remote_node);

  RCLCPP_INFO(this->get_logger(), "Waiting for %s to expose %s...",
    remote_node.c_str(), remote_parameter.c_str());
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      throw std::runtime_error("Interrupted while waiting for robot_description parameter");
    }
    RCLCPP_INFO(this->get_logger(), "Still waiting for %s...", remote_node.c_str());
  }

  const auto parameters = client->get_parameters({remote_parameter});
  if (parameters.empty() || parameters.front().get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    throw std::runtime_error("robot_description parameter is missing or not a string");
  }

  const auto robot_description = parameters.front().as_string();
  const auto limits = parse_joint_limits(robot_description);
  if (limits.size() < 3) {
    throw std::runtime_error("Failed to parse at least 3 revolute joint limits from robot_description");
  }

  loaded_revolute_joint_count_ = std::min<std::size_t>(joint_limits_.size(), limits.size());
  for (std::size_t i = 0; i < loaded_revolute_joint_count_; ++i) {
    joint_limits_[i] = limits[i];
  }

  RCLCPP_INFO(this->get_logger(),
    "Loaded IK limits for %zu revolute joints. First three: q1[%.3f, %.3f], q2[%.3f, %.3f], q3[%.3f, %.3f]",
    loaded_revolute_joint_count_,
    joint_limits_[0].lower, joint_limits_[0].upper,
    joint_limits_[1].lower, joint_limits_[1].upper,
    joint_limits_[2].lower, joint_limits_[2].upper);

  if (loaded_revolute_joint_count_ < 6) {
    RCLCPP_WARN(
      this->get_logger(),
      "6DOF IK services require 6 revolute joints, but only %zu were found in URDF.",
      loaded_revolute_joint_count_);
  }
}

std::vector<IkSolverNode::JointLimit> IkSolverNode::parse_joint_limits(
  const std::string & robot_description)
{
  std::vector<JointLimit> limits;
  std::size_t search_position = 0;

  while (search_position < robot_description.size()) {
    const auto joint_start = robot_description.find("<joint", search_position);
    if (joint_start == std::string::npos) {
      break;
    }

    const auto joint_end = robot_description.find("</joint>", joint_start);
    if (joint_end == std::string::npos) {
      break;
    }

    const auto joint_block = robot_description.substr(joint_start, joint_end - joint_start);
    search_position = joint_end + 8;

    if (!contains_revolute_type(joint_block)) {
      continue;
    }

    const auto lower_value = extract_attribute(joint_block, "lower");
    const auto upper_value = extract_attribute(joint_block, "upper");
    if (!lower_value || !upper_value) {
      continue;
    }

    limits.push_back(JointLimit{std::stod(*lower_value), std::stod(*upper_value)});
  }

  return limits;
}

std::optional<std::string> IkSolverNode::extract_attribute(
  const std::string & block,
  const std::string & attribute_name)
{
  for (const char quote : {'"', '\''}) {
    const std::string needle = attribute_name + "=" + quote;
    const auto value_start = block.find(needle);
    if (value_start == std::string::npos) {
      continue;
    }

    const auto token_start = value_start + needle.size();
    const auto token_end = block.find(quote, token_start);
    if (token_end == std::string::npos) {
      continue;
    }

    return trim_copy(block.substr(token_start, token_end - token_start));
  }

  return std::nullopt;
}

bool IkSolverNode::contains_revolute_type(const std::string & block)
{
  return block.find("type=\"revolute\"") != std::string::npos ||
         block.find("type='revolute'") != std::string::npos;
}

double IkSolverNode::clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

double IkSolverNode::wrap_to_pi(double angle)
{
  return std::remainder(angle, kTwoPi);
}

double IkSolverNode::shortest_angular_distance(double from, double to)
{
  return std::remainder(to - from, kTwoPi);
}

bool IkSolverNode::nearly_equal(
  const std::array<double, 3> & lhs,
  const std::array<double, 3> & rhs)
{
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (std::abs(shortest_angular_distance(lhs[i], rhs[i])) > 1e-5) {
      return false;
    }
  }
  return true;
}

std::optional<std::array<double, 3>> IkSolverNode::fit_solution_to_limits(
  const std::array<double, 3> & raw_solution,
  const std::array<double, 3> * reference) const
{
  std::array<double, 3> normalized{};

  for (std::size_t joint_index = 0; joint_index < raw_solution.size(); ++joint_index) {
    const auto & limit = joint_limits_[joint_index];
    double best_candidate = 0.0;
    double best_score = std::numeric_limits<double>::infinity();
    bool found_candidate = false;

    for (int shift = -4; shift <= 4; ++shift) {
      const double candidate = raw_solution[joint_index] + static_cast<double>(shift) * kTwoPi;
      if (candidate < limit.lower - kTolerance || candidate > limit.upper + kTolerance) {
        continue;
      }

      const double score = reference == nullptr ?
        std::abs(candidate - raw_solution[joint_index]) :
        std::abs(shortest_angular_distance((*reference)[joint_index], candidate));

      if (score < best_score) {
        best_score = score;
        best_candidate = candidate;
        found_candidate = true;
      }
    }

    if (!found_candidate) {
      return std::nullopt;
    }

    normalized[joint_index] = best_candidate;
  }

  return normalized;
}

std::vector<IkSolverNode::CandidateSolution> IkSolverNode::compute_solutions(
  double x,
  double y,
  double z,
  const std::array<double, 3> * reference) const
{
  std::vector<CandidateSolution> solutions;
  if (loaded_revolute_joint_count_ < 3) {
    return solutions;
  }

  const double rho = std::hypot(x, y); // Hypot pocita preponu, ano zacal som velkym pismenom. Ano som v strese
  const double z_offset = z - kLink1Offset;
  const double base_angle = std::atan2(y, x);

  for (int sign : {1, -1}) {
    const double signed_rho = static_cast<double>(sign) * rho;
    const double q1_raw = sign > 0 ? base_angle : wrap_to_pi(base_angle + kPi);

    const double cos_q3 = clamp(
      (signed_rho * signed_rho + z_offset * z_offset - kLink2Length * kLink2Length -
      kLink3Length * kLink3Length) /
      (2.0 * kLink2Length * kLink3Length),
      -1.0, 1.0);

    if (cos_q3 < -1.0 - 1e-8 || cos_q3 > 1.0 + 1e-8) {
      continue;
    }

    const double q3_abs = std::acos(cos_q3);
    for (double q3_raw : {q3_abs, -q3_abs}) {
      const double q2_raw = std::atan2(z_offset, signed_rho) -
        std::atan2(kLink3Length * std::sin(q3_raw),
        kLink2Length + kLink3Length * std::cos(q3_raw));

      const std::array<double, 3> raw_solution{{q1_raw, q2_raw, q3_raw}};
      const auto normalized_solution = fit_solution_to_limits(raw_solution, reference);
      if (!normalized_solution) {
        continue;
      }

      const CandidateSolution candidate{*normalized_solution};
      bool duplicate = false;
      for (const auto & existing : solutions) {
        if (nearly_equal(existing.joints, candidate.joints)) {
          duplicate = true;
          break;
        }
      }

      if (!duplicate) {
        solutions.push_back(candidate);
      }
    }
  }

  return solutions;
}

std::array<std::array<double, 3>, 3> IkSolverNode::quaternion_to_rotation(
  const geometry_msgs::msg::Pose & pose,
  bool & valid)
{
  const double qx = pose.orientation.x;
  const double qy = pose.orientation.y;
  const double qz = pose.orientation.z;
  const double qw = pose.orientation.w;
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);

  valid = norm > 1e-12;
  if (!valid) {
    return {{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};
  }

  const double x = qx / norm;
  const double y = qy / norm;
  const double z = qz / norm;
  const double w = qw / norm;

  return {{
    {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)},
    {2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)},
    {2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)}
  }};
}

std::array<std::array<double, 3>, 3> IkSolverNode::rotation_transpose(
  const std::array<std::array<double, 3>, 3> & matrix)
{
  return {{
    {matrix[0][0], matrix[1][0], matrix[2][0]},
    {matrix[0][1], matrix[1][1], matrix[2][1]},
    {matrix[0][2], matrix[1][2], matrix[2][2]}
  }};
}

std::array<std::array<double, 3>, 3> IkSolverNode::rotation_multiply(
  const std::array<std::array<double, 3>, 3> & lhs,
  const std::array<std::array<double, 3>, 3> & rhs)
{
  std::array<std::array<double, 3>, 3> result{{
    {{0.0, 0.0, 0.0}},
    {{0.0, 0.0, 0.0}},
    {{0.0, 0.0, 0.0}}
  }};

  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      for (std::size_t k = 0; k < 3; ++k) {
        result[i][j] += lhs[i][k] * rhs[k][j];
      }
    }
  }

  return result;
}

std::vector<std::array<double, 3>> IkSolverNode::solve_wrist_zyz(
  const std::array<std::array<double, 3>, 3> & rotation_36)
{
  std::vector<std::array<double, 3>> wrist_solutions;

  const double c5 = clamp(rotation_36[2][2], -1.0, 1.0);
  const double q5_abs = std::acos(c5);

  for (double q5 : {q5_abs, -q5_abs}) {
    const double s5 = std::sin(q5);
    double q4 = 0.0;
    double q6 = 0.0;

    if (std::abs(s5) > 1e-8) {
      q4 = std::atan2(rotation_36[1][2] / s5, rotation_36[0][2] / s5);
      q6 = std::atan2(rotation_36[2][1] / s5, -rotation_36[2][0] / s5);
    } else {
      q4 = 0.0;
      q6 = std::atan2(rotation_36[1][0], rotation_36[0][0]);
    }

    wrist_solutions.push_back({{wrap_to_pi(q4), wrap_to_pi(q5), wrap_to_pi(q6)}});
  }

  return wrist_solutions;
}

std::optional<std::array<double, 6>> IkSolverNode::fit_solution_to_limits_6d(
  const std::array<double, 6> & raw_solution,
  const std::array<double, 6> * reference) const
{
  std::array<double, 6> normalized{};

  for (std::size_t joint_index = 0; joint_index < raw_solution.size(); ++joint_index) {
    const auto & limit = joint_limits_[joint_index];
    double best_candidate = 0.0;
    double best_score = std::numeric_limits<double>::infinity();
    bool found_candidate = false;

    for (int shift = -4; shift <= 4; ++shift) {
      const double candidate = raw_solution[joint_index] + static_cast<double>(shift) * kTwoPi;
      if (candidate < limit.lower - kTolerance || candidate > limit.upper + kTolerance) {
        continue;
      }

      const double score = reference == nullptr ?
        std::abs(candidate - raw_solution[joint_index]) :
        std::abs(shortest_angular_distance((*reference)[joint_index], candidate));

      if (score < best_score) {
        best_score = score;
        best_candidate = candidate;
        found_candidate = true;
      }
    }

    if (!found_candidate) {
      return std::nullopt;
    }

    normalized[joint_index] = best_candidate;
  }

  return normalized;
}

bool IkSolverNode::nearly_equal_6d(
  const std::array<double, 6> & lhs,
  const std::array<double, 6> & rhs)
{
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (std::abs(shortest_angular_distance(lhs[i], rhs[i])) > 1e-5) {
      return false;
    }
  }
  return true;
}

std::vector<IkSolverNode::CandidateSolution6> IkSolverNode::compute_solutions_6d(
  const geometry_msgs::msg::Pose & target_pose,
  const std::array<double, 6> * reference) const
{
  std::vector<CandidateSolution6> solutions;
  if (loaded_revolute_joint_count_ < 6) {
    return solutions;
  }

  bool valid_quaternion = false;
  const auto rotation_06 = quaternion_to_rotation(target_pose, valid_quaternion);
  if (!valid_quaternion) {
    return solutions;
  }

  //tu vyberam z stlpec z rotacnej matice. (decoupling)
  const std::array<double, 3> z_tool{{rotation_06[0][2], rotation_06[1][2], rotation_06[2][2]}};
  const std::array<double, 3> wrist_center{{
    target_pose.position.x - kArm6ToolOffset * z_tool[0],
    target_pose.position.y - kArm6ToolOffset * z_tool[1],
    target_pose.position.z - kArm6ToolOffset * z_tool[2]
  }};
//
  const double rho = std::hypot(wrist_center[0], wrist_center[1]);
  const double z_offset = wrist_center[2];
  const double base_angle = std::atan2(wrist_center[1], wrist_center[0]);

  for (int sign : {1, -1}) {
    const double signed_rho = static_cast<double>(sign) * rho;
    const double q1_raw = sign > 0 ? base_angle : wrap_to_pi(base_angle + kPi);

    const double cos_q3 = clamp(
      (signed_rho * signed_rho + z_offset * z_offset -
      kArm6Link2ToJoint3 * kArm6Link2ToJoint3 -
      kArm6Joint3ToWrist * kArm6Joint3ToWrist) /
      (2.0 * kArm6Link2ToJoint3 * kArm6Joint3ToWrist),
      -1.0, 1.0);

    if (cos_q3 < -1.0 - 1e-8 || cos_q3 > 1.0 + 1e-8) {
      continue;
    }

    const double q3_abs = std::acos(cos_q3);
    for (double q3_raw : {q3_abs, -q3_abs}) {
      const double q2_raw = std::atan2(z_offset, signed_rho) -
        std::atan2(
        kArm6Joint3ToWrist * std::sin(q3_raw),
        kArm6Link2ToJoint3 + kArm6Joint3ToWrist * std::cos(q3_raw));

      const double c1 = std::cos(q1_raw);
      const double s1 = std::sin(q1_raw);
      const double c23 = std::cos(q2_raw + q3_raw);
      const double s23 = std::sin(q2_raw + q3_raw);

      const std::array<std::array<double, 3>, 3> rotation_03{{
        {c1 * c23, -s1, c1 * s23},
        {s1 * c23, c1, s1 * s23},
        {-s23, 0.0, c23}
      }};

      const auto rotation_36 = rotation_multiply(rotation_transpose(rotation_03), rotation_06);
      const auto wrist_solutions = solve_wrist_zyz(rotation_36);

      for (const auto & wrist : wrist_solutions) {
        const std::array<double, 6> raw{{q1_raw, q2_raw, q3_raw, wrist[0], wrist[1], wrist[2]}};
        const auto normalized = fit_solution_to_limits_6d(raw, reference);
        if (!normalized) {
          continue;
        }

        const CandidateSolution6 candidate{*normalized};
        bool duplicate = false;
        for (const auto & existing : solutions) {
          if (nearly_equal_6d(existing.joints, candidate.joints)) {
            duplicate = true;
            break;
          }
        }

        if (!duplicate) {
          solutions.push_back(candidate);
        }
      }
    }
  }

  return solutions;
}

void IkSolverNode::handle_all_solutions(
  const std::shared_ptr<block1_dzvonar::srv::IkAllSolutions::Request> request,
  std::shared_ptr<block1_dzvonar::srv::IkAllSolutions::Response> response)
{
  const auto solutions = compute_solutions(request->x, request->y, request->z);
  if (solutions.empty()) {
    response->success = false;
    response->message = "Target is unreachable or all IK solutions violate joint limits";
    response->solution_count = 0;
    response->joint_solutions.clear();
    return;
  }

  response->success = true;
  response->solution_count = static_cast<uint8_t>(solutions.size());
  response->joint_solutions.clear();
  response->joint_solutions.reserve(solutions.size() * 3);

  for (const auto & solution : solutions) {
    response->joint_solutions.push_back(solution.joints[0]);
    response->joint_solutions.push_back(solution.joints[1]);
    response->joint_solutions.push_back(solution.joints[2]);
  }

  response->message = "Found " + std::to_string(solutions.size()) + " valid IK solutions";
}

void IkSolverNode::handle_best_solution(
  const std::shared_ptr<block1_dzvonar::srv::IkBestSolution::Request> request,
  std::shared_ptr<block1_dzvonar::srv::IkBestSolution::Response> response)
{
  const std::array<double, 3> current{{request->current_j1, request->current_j2, request->current_j3}};
  const auto solutions = compute_solutions(request->x, request->y, request->z, &current);
  if (solutions.empty()) {
    response->success = false;
    response->message = "No valid IK solution found for the requested target";
    return;
  }

  const auto best = std::min_element(
    solutions.begin(), solutions.end(),
    [&current](const CandidateSolution & lhs, const CandidateSolution & rhs) {
      double lhs_cost = 0.0;
      double rhs_cost = 0.0;
      for (std::size_t index = 0; index < current.size(); ++index) {
        const double lhs_delta = shortest_angular_distance(current[index], lhs.joints[index]);
        const double rhs_delta = shortest_angular_distance(current[index], rhs.joints[index]);
        lhs_cost += lhs_delta * lhs_delta;
        rhs_cost += rhs_delta * rhs_delta;
      }
      return lhs_cost < rhs_cost;
    });

  response->success = true;
  response->message = "Best IK solution selected";
  response->joints[0] = best->joints[0];
  response->joints[1] = best->joints[1];
  response->joints[2] = best->joints[2];
}

void IkSolverNode::handle_6d_all_solutions(
  const std::shared_ptr<block1_dzvonar::srv::Ik6AllSolutions::Request> request,
  std::shared_ptr<block1_dzvonar::srv::Ik6AllSolutions::Response> response)
{
  if (loaded_revolute_joint_count_ < 6) {
    response->success = false;
    response->message = "URDF does not expose 6 revolute joints for 6DOF IK";
    response->solution_count = 0;
    response->joint_solutions.clear();
    return;
  }

  const auto solutions = compute_solutions_6d(request->target_pose);
  if (solutions.empty()) {
    response->success = false;
    response->message = "No valid 6DOF IK solution found for requested pose";
    response->solution_count = 0;
    response->joint_solutions.clear();
    return;
  }

  response->success = true;
  response->solution_count = static_cast<uint8_t>(solutions.size());
  response->joint_solutions.clear();
  response->joint_solutions.reserve(solutions.size() * 6);

  for (const auto & solution : solutions) {
    response->joint_solutions.insert(
      response->joint_solutions.end(),
      solution.joints.begin(),
      solution.joints.end());
  }

  response->message = "Found " + std::to_string(solutions.size()) + " valid 6DOF IK solutions";
}

void IkSolverNode::handle_6d_best_solution(
  const std::shared_ptr<block1_dzvonar::srv::Ik6BestSolution::Request> request,
  std::shared_ptr<block1_dzvonar::srv::Ik6BestSolution::Response> response)
{
  if (loaded_revolute_joint_count_ < 6) {
    response->success = false;
    response->message = "URDF does not expose 6 revolute joints for 6DOF IK";
    return;
  }

  std::array<double, 6> current{};
  for (std::size_t i = 0; i < current.size(); ++i) {
    current[i] = request->current_joints[i];
  }

  const auto solutions = compute_solutions_6d(request->target_pose, &current);
  if (solutions.empty()) {
    response->success = false;
    response->message = "No valid 6DOF IK solution found for requested pose";
    return;
  }

  const auto best = std::min_element(
    solutions.begin(), solutions.end(),
    [&current](const CandidateSolution6 & lhs, const CandidateSolution6 & rhs) {
      double lhs_cost = 0.0;
      double rhs_cost = 0.0;
      for (std::size_t i = 0; i < current.size(); ++i) {
        const double lhs_delta = shortest_angular_distance(current[i], lhs.joints[i]);
        const double rhs_delta = shortest_angular_distance(current[i], rhs.joints[i]);
        lhs_cost += lhs_delta * lhs_delta;
        rhs_cost += rhs_delta * rhs_delta;
      }
      return lhs_cost < rhs_cost;
    });

  response->success = true;
  response->message = "Best 6DOF IK solution selected";
  for (std::size_t i = 0; i < response->joints.size(); ++i) {
    response->joints[i] = best->joints[i];
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IkSolverNode>();

  try {
    node->initialize_from_robot_description();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize IK solver: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
