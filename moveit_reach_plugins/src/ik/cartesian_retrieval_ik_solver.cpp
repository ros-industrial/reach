/*
 * Copyright 2022 PickNik, Inc.
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
#include "moveit_reach_plugins/ik/cartesian_retrieval_ik_solver.h"

#include <algorithm>

namespace {

template <typename T>
T clamp(const T& val, const T& low, const T& high) {
  return std::max(low, std::min(val, high));
}

}  // namespace

namespace moveit_reach_plugins {
namespace ik {

CartesianRetrievalIKSolver::CartesianRetrievalIKSolver() : MoveItIKSolver() {}

bool CartesianRetrievalIKSolver::initialize(
    std::string& name, rclcpp::Node::SharedPtr node,
    const std::shared_ptr<const moveit::core::RobotModel> model) {
  // initialize base class
  if (!MoveItIKSolver::initialize(name, node, model)) {
    RCLCPP_ERROR(LOGGER,
                 "Failed to initialize CartesianRetrievalIKSolver plugin");
    return false;
  }

  // get parameters for cartesian path computation
  try {
    if (!node->get_parameter("ik_solver_config.retrieval_distance",
                             retrieval_path_length_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.retrieval_distance' ");
      return false;
    }
    // make sure it is positive to follow solvers logic
    retrieval_path_length_ = std::abs(double(retrieval_path_length_));
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(LOGGER, ex.what());
    return false;
  }

  // output message about successful initialization
  RCLCPP_INFO(LOGGER,
              "Successfully initialized CartesianRetrievalIKSolver plugin");
  return true;
}

std::optional<double> CartesianRetrievalIKSolver::solveIKFromSeed(
    const Eigen::Isometry3d& target, const std::map<std::string, double>& seed,
    std::vector<double>& solution) {
  return MoveItIKSolver::solveIKFromSeed(target, seed, solution);
}

std::vector<std::string> CartesianRetrievalIKSolver::getJointNames() const {
  return MoveItIKSolver::getJointNames();
}

}  // namespace ik
}  // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::ik::CartesianRetrievalIKSolver,
                       reach::plugins::IKSolverBase)
