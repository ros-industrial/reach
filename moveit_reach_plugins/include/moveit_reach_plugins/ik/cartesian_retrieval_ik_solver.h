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
#ifndef MOVEIT_REACH_PLUGINS_IK_CARTESIAN_RETRIEVAL_IK_SOLVER_H
#define MOVEIT_REACH_PLUGINS_IK_CARTESIAN_RETRIEVAL_IK_SOLVER_H

#include "moveit_ik_solver.h"

#include <pluginlib/class_loader.hpp>
#include <reach_core/plugins/evaluation_base.h>

// PlanningScene
#include <moveit_msgs/msg/planning_scene.hpp>

// cartesian interpolator include
#include <moveit/robot_state/cartesian_interpolator.h>

namespace planning_scene {
class PlanningScene;
typedef std::shared_ptr<PlanningScene> PlanningScenePtr;
}  // namespace planning_scene

namespace moveit_reach_plugins {
// using same LOGGER as MoveItIKSolver --> moveit_reach_plugins.MoveItIKSolver
namespace ik {

class CartesianRetrievalIKSolver : public MoveItIKSolver {
 public:
  CartesianRetrievalIKSolver();

  ~CartesianRetrievalIKSolver() {
    eval_.reset();
    model_.reset();
    scene_pub_.reset();
    scene_.reset();
  }

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
  // distance to retrieve from ik solution in [m]
  double retrieval_path_length_;
  double jump_threshold_;
  double max_eef_step_;
  std::string tool_frame_;
};

}  // namespace ik
}  // namespace moveit_reach_plugins

#endif  // MOVEIT_REACH_PLUGINS_IK_CARTESIAN_RETRIEVAL_IK_SOLVER_H
