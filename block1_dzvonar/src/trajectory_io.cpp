#include "block1_Dzvonar/trajectory_io.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

void save_teach_point(
  int id,
  const std::vector<double>& positions,
  double max_velocity,
  const std::string& filename)
{
  std::ofstream file(filename, std::ios::app);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open file: " + filename);
  }
  file << id;
  for (const auto& p : positions) {
    file << "," << p;
  }
  file << "," << max_velocity << "\n";
}

std::vector<TeachPoint> load_teach_points(const std::string& filename)
{
  std::vector<TeachPoint> points;
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open file: " + filename);
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string token;
    TeachPoint pt;

    std::getline(ss, token, ',');
    pt.id = std::stoi(token);

    // read until last token (max_velocity)
    std::vector<std::string> rest;
    while (std::getline(ss, token, ',')) {
      rest.push_back(token);
    }
    // last is max_velocity, rest are positions
    for (size_t i = 0; i + 1 < rest.size(); i++) {
      pt.positions.push_back(std::stod(rest[i]));
    }
    pt.max_velocity = std::stod(rest.back());
    points.push_back(pt);
  }
  return points;
}