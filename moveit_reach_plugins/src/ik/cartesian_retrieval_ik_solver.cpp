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

#include "moveit_reach_plugins/utils.h"
#include "tf2_eigen/tf2_eigen.h"

#include <algorithm>

#include <reach_core/utils/general_utils.h>

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
    if (!node->get_parameter("ik_solver_config.max_eef_step", max_eef_step_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.max_eef_step' ");
      return false;
    }
    if (!node->get_parameter("ik_solver_config.jump_threshold",
                             jump_threshold_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.jump_threshold' ");
      return false;
    }
    if (!node->get_parameter("ik_solver_config.tool_frame", tool_frame_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.jump_threshold' ");
      return false;
    }
    // make sure it is positive to follow solvers logic
    retrieval_path_length_ = std::abs(double(retrieval_path_length_));
    max_eef_step_ = std::abs(double(max_eef_step_));
    jump_threshold_ = std::abs(double(jump_threshold_));
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
    std::vector<double>& solution, std::vector<double>& joint_space_trajectory,
    std::vector<double>& cartesian_space_waypoints, double& fraction) {
  moveit::core::RobotState state(model_);

  const std::vector<std::string>& joint_names =
      jmg_->getActiveJointModelNames();

  std::vector<double> seed_subset;
  if (!utils::transcribeInputMap(seed, joint_names, seed_subset)) {
    RCLCPP_ERROR_STREAM(
        LOGGER, __FUNCTION__ << ": failed to transcribe input pose map");
    return {};
  }

  state.setJointGroupPositions(jmg_, seed_subset);
  state.update();

  //  const static int SOLUTION_ATTEMPTS = 3;
  // use default timeout for the ik solver
  const static double SOLUTION_TIMEOUT = 0.0;

  if (state.setFromIK(jmg_, target, SOLUTION_TIMEOUT,
                      std::bind(&MoveItIKSolver::isIKSolutionValid, this,
                                std::placeholders::_1, std::placeholders::_2,
                                std::placeholders::_3))) {
    solution.clear();
    state.copyJointGroupPositions(jmg_, solution);

    // Convert back to map
    std::map<std::string, double> solution_map;
    for (std::size_t i = 0; i < solution.size(); ++i) {
      solution_map.emplace(joint_names[i], solution[i]);
    }

    // compute retrieval path
    std::vector<std::shared_ptr<moveit::core::RobotState>> traj;
    Eigen::Isometry3d retrieval_target =
        target * Eigen::Translation3d(0.0, 0.0, -retrieval_path_length_);

    double f = moveit::core::CartesianInterpolator::computeCartesianPath(
        &state, jmg_, traj, state.getLinkModel(tool_frame_), retrieval_target,
        true, moveit::core::MaxEEFStep(max_eef_step_),
        moveit::core::JumpThreshold(jump_threshold_, jump_threshold_),
        std::bind(&MoveItIKSolver::isIKSolutionValid, this,
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3));
    fraction = f;

    if (f != 0.0) {
      const size_t trajectory_size = traj.size();
      const size_t joints_size = joint_names.size();
      size_t joint_space_idx = 0;
      size_t cartesian_space_idx = 0;
      const size_t joint_space_traj_size = trajectory_size * joints_size;
      const size_t cartesian_space_wpts_size = trajectory_size * 7;
      joint_space_trajectory.resize(joint_space_traj_size);
      cartesian_space_waypoints.resize(cartesian_space_wpts_size);

      for (size_t i = 0;
           i < trajectory_size && joint_space_idx < joint_space_traj_size &&
           cartesian_space_idx < cartesian_space_wpts_size;
           ++i) {
        std::vector<double> joint_positions_tmp;
        traj[i]->copyJointGroupPositions(jmg_, joint_positions_tmp);
        geometry_msgs::msg::Pose pose_tmp =
            tf2::toMsg(traj[i]->getFrameTransform(tool_frame_));

        for (size_t j = 0; j < joints_size; ++j) {
          joint_space_trajectory[joint_space_idx + j] = joint_positions_tmp[j];
        }
        joint_space_idx += joints_size;

        cartesian_space_waypoints[cartesian_space_idx + 0] =
            pose_tmp.position.x;
        cartesian_space_waypoints[cartesian_space_idx + 1] =
            pose_tmp.position.y;
        cartesian_space_waypoints[cartesian_space_idx + 2] =
            pose_tmp.position.z;
        cartesian_space_waypoints[cartesian_space_idx + 3] =
            pose_tmp.orientation.x;
        cartesian_space_waypoints[cartesian_space_idx + 4] =
            pose_tmp.orientation.y;
        cartesian_space_waypoints[cartesian_space_idx + 5] =
            pose_tmp.orientation.z;
        cartesian_space_waypoints[cartesian_space_idx + 6] =
            pose_tmp.orientation.w;
        cartesian_space_idx += 7;
      }

      return fraction * eval_->calculateScore(solution_map);
    } else {
      // make sure trajectory is empty on exit
      cartesian_space_waypoints.clear();
      joint_space_trajectory.clear();
      return {};
    }
  } else {
    fraction = 0.0;
    return {};
  }
}

}  // namespace ik
}  // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::ik::CartesianRetrievalIKSolver,
                       reach::plugins::IKSolverBase)
