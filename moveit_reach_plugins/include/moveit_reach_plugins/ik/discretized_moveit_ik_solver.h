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
#ifndef MOVEIT_REACH_PLUGINS_IK_DISCRETIZED_MOVEIT_IK_SOLVER_H
#define MOVEIT_REACH_PLUGINS_IK_DISCRETIZED_MOVEIT_IK_SOLVER_H

#include "moveit_ik_solver.h"

namespace moveit_reach_plugins {

namespace ik {

class DiscretizedMoveItIKSolver : public MoveItIKSolver {
 public:
  DiscretizedMoveItIKSolver();

  ~DiscretizedMoveItIKSolver() = default;

  virtual bool initialize(
      std::string& name, rclcpp::Node::SharedPtr node,
      const std::shared_ptr<const moveit::core::RobotModel> model) override;

  virtual std::optional<double> solveIKFromSeed(
      const Eigen::Isometry3d& target,
      const std::map<std::string, double>& seed, std::vector<double>& solution,
      std::vector<double>& joint_space_trajectory,
      std::vector<double>& cartesian_space_waypoints,
      double& fraction) override;

 protected:
  double dt_;
};

}  // namespace ik
}  // namespace moveit_reach_plugins

#endif  // MOVEIT_REACH_PLUGINS_IK_DISCRETIZED_MOVEIT_IK_SOLVER_H
