/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "moveit_reach_plugins/ik/discretized_moveit_ik_solver.h"

#include <algorithm>

namespace {

template <typename T>
T clamp(const T& val, const T& low, const T& high) {
  return std::max(low, std::min(val, high));
}

}  // namespace

namespace moveit_reach_plugins {
namespace ik {

DiscretizedMoveItIKSolver::DiscretizedMoveItIKSolver() : MoveItIKSolver() {}

bool DiscretizedMoveItIKSolver::initialize(
    std::string& name, rclcpp::Node::SharedPtr node,
    const std::shared_ptr<const moveit::core::RobotModel> model) {
  if (!MoveItIKSolver::initialize(name, node, model)) {
    RCLCPP_ERROR(LOGGER, "Failed to initialize MoveItIKSolver plugin");
    return false;
  }

  try {
    if (!node->get_parameter("ik_solver_config.discretization_angle", dt_)) {
      return false;
    }
    dt_ = std::abs(double(dt_));
    double clamped_dt = clamp<double>(dt_, 0.0, M_PI);
    if (std::abs(dt_ - clamped_dt) > 1.0e-6) {
      RCLCPP_WARN_STREAM(
          LOGGER,
          "Clamping discretization angle between 0 and pi; new value is "
              << clamped_dt);
    }
    dt_ = clamped_dt;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(LOGGER, ex.what());
    return false;
  }

  RCLCPP_INFO_STREAM(
      LOGGER, "Successfully initialized DiscretizedMoveItIKSolver plugin");
  return true;
}

std::optional<double> DiscretizedMoveItIKSolver::solveIKFromSeed(
    const Eigen::Isometry3d& target, const std::map<std::string, double>& seed,
    std::vector<double>& solution, std::vector<double>& joint_space_trajectory,
    std::vector<double>& cartesian_space_waypoints, double& fraction) {
  // Calculate the number of discretizations necessary to achieve discretization
  // angle
  const static int n_discretizations = int((2.0 * M_PI) / dt_);

  // Set up containers for the best solution to be saved into the database
  std::vector<double> best_solution;
  double best_score = 0;

  for (int i = 0; i < n_discretizations; ++i) {
    Eigen::Isometry3d discretized_target(
        target * Eigen::AngleAxisd(double(i) * dt_, Eigen::Vector3d::UnitZ()));
    std::vector<double> tmp_solution;
    std::optional<double> score = MoveItIKSolver::solveIKFromSeed(
        discretized_target, seed, tmp_solution, joint_space_trajectory,
        cartesian_space_waypoints, fraction);
    if (score.has_value() && (score.value() > best_score)) {
      best_score = score.value();
      best_solution = std::move(tmp_solution);
    } else {
      continue;
    }
  }

  if (best_score > 0) {
    solution = std::move(best_solution);
    return std::optional<double>(best_score);
  } else {
    fraction = 0.0;
    return {};
  }
}

}  // namespace ik
}  // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::ik::DiscretizedMoveItIKSolver,
                       reach::plugins::IKSolverBase)
