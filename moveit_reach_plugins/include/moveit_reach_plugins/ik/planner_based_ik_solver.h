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

/* Authors: Lovro Ivanov, @livanov93
   Desc:
*/
#ifndef MOVEIT_REACH_PLUGINS_IK_PLANNER_BASED_IK_SOLVER_H
#define MOVEIT_REACH_PLUGINS_IK_PLANNER_BASED_IK_SOLVER_H

#include "moveit_ik_solver.h"

#include <pluginlib/class_loader.hpp>
#include <reach_core/plugins/evaluation_base.h>

// PlanningScene
#include <moveit_msgs/msg/planning_scene.hpp>

// cartesian interpolator include
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit_msgs/msg/constraints.hpp>

namespace planning_scene {
class PlanningScene;
typedef std::shared_ptr<PlanningScene> PlanningScenePtr;
}  // namespace planning_scene

namespace moveit_reach_plugins {
// using same LOGGER as MoveItIKSolver --> moveit_reach_plugins.MoveItIKSolver
namespace ik {

class PlannerBasedIKSolver : public MoveItIKSolver {
 public:
  // https://github.com/ros-planning/moveit_task_constructor/blob/60229db010ea305296bc1c90d04faa3e4dacd976/core/include/moveit/task_constructor/solvers/pipeline_planner.h#L58-L64
  struct PlanningSpecs {
    moveit::core::RobotModelConstPtr _robot_model;
    std::string _namespace{"ompl"};
    std::string _planning_pipeline{"ompl"};
    std::string _planning_adapter_param{"request_adapters"};
  };

  PlannerBasedIKSolver();

  ~PlannerBasedIKSolver() {
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
  // https://github.com/ros-planning/moveit_task_constructor/blob/60229db010ea305296bc1c90d04faa3e4dacd976/core/include/moveit/task_constructor/solvers/pipeline_planner.h#L66-L72
  static planning_pipeline::PlanningPipelinePtr create(
      const rclcpp::Node::SharedPtr& node,
      const moveit::core::RobotModelConstPtr& model) {
    PlanningSpecs spec;
    spec._robot_model = model;
    return create(node, spec);
  }

  static planning_pipeline::PlanningPipelinePtr create(
      const rclcpp::Node::SharedPtr& node, const PlanningSpecs& spec);

  // distance to retrieve from ik solution in [m]
  double retrieval_path_length_;
  double jump_threshold_;
  double max_eef_step_;
  std::string tool_frame_;

  bool display_motion_plans_;
  bool publish_planning_requests_;
  uint num_planning_attempts_;
  double max_velocity_scaling_factor_;
  double max_acceleration_scaling_factor_;
  moveit_msgs::msg::WorkspaceParameters workspace_parameter_;
  // planner id
  std::string planner_id_;
  double goal_joint_tolerance_;
  double goal_position_tolerance_;
  double goal_orientation_tolerance_;
  double allowed_planning_time_;

  std::string pipeline_name_;
  planning_pipeline::PlanningPipelinePtr planner_;
};

}  // namespace ik
}  // namespace moveit_reach_plugins

#endif  // MOVEIT_REACH_PLUGINS_IK_PLANNER_BASED_IK_SOLVER_H
