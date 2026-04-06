#pragma once

#include <string>
#include <vector>

// Free function - OOP: not part of any node class
void save_teach_point(
  int id,
  const std::vector<double>& positions,
  double max_velocity,
  const std::string& filename = "trajectory.csv");

struct TeachPoint {
  int id;
  std::vector<double> positions;
  double max_velocity;
};

std::vector<TeachPoint> load_teach_points(
  const std::string& filename = "trajectory.csv");