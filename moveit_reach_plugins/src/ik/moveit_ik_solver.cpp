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
#include "moveit_reach_plugins/ik/moveit_ik_solver.h"

#include "moveit_reach_plugins/utils.h"

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/planning_scene.hpp>

namespace moveit_reach_plugins {
namespace ik {

const static std::string PACKAGE = "reach_core";
const static std::string EVAL_PLUGIN_BASE = "reach::plugins::EvaluationBase";

MoveItIKSolver::MoveItIKSolver()
    : reach::plugins::IKSolverBase(),
      class_loader_(PACKAGE, EVAL_PLUGIN_BASE) {}

bool MoveItIKSolver::initialize(
    std::string& name, rclcpp::Node::SharedPtr node,
    const std::shared_ptr<const moveit::core::RobotModel> model) {
  node_ = node;

  std::string planning_group;

  if (!node->get_parameter("ik_solver_config.planning_group", planning_group) ||
      !node->get_parameter("ik_solver_config.distance_threshold",
                           distance_threshold_) ||
      !node->get_parameter("ik_solver_config.collision_mesh_package",
                           collision_mesh_package_) ||
      !node->get_parameter("ik_solver_config.collision_mesh_filename_path",
                           collision_mesh_filename_path_) ||
      !node->get_parameter("ik_solver_config.collision_mesh_frame",
                           collision_mesh_frame_) ||
      !node->get_parameter("ik_solver_config.touch_links", touch_links_) ||
      !node->get_parameter("ik_solver_config.evaluation_plugin.name",
                           evaluation_plugin_name_)) {
    RCLCPP_ERROR(LOGGER,
                 "MoveIt IK Solver Plugin is missing one or more configuration "
                 "parameters");
    return false;
  }

  if (std::find(touch_links_.begin(), touch_links_.end(), "") !=
      touch_links_.end()) {
    touch_links_.clear();
  }

  try {
    eval_ = class_loader_.createSharedInstance(evaluation_plugin_name_);
  } catch (const pluginlib::ClassLoaderException& ex) {
    RCLCPP_ERROR_STREAM(LOGGER, ex.what());
  }
  try {
    if (!eval_->initialize(evaluation_plugin_name_, node, model)) {
      RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialize evaluation plugin");
      return false;
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(LOGGER, ex.what());
    return false;
  }

  //    model_ = moveit::planning_interface::getSharedRobotModelLoader(node,
  //    "robot_description")->getModel();
  model_ = model;

  if (!model_) {
    RCLCPP_ERROR(LOGGER, "Failed to initialize robot model pointer");
    return false;
  }

  jmg_ = model_->getJointModelGroup(planning_group);
  if (!jmg_) {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to get joint model group for '"
                                    << planning_group << "'");
    return false;
  }

  scene_ = std::make_shared<planning_scene::PlanningScene>(model_);

  // Check that the input collision mesh frame exists
  if (!scene_->knowsFrameTransform(collision_mesh_frame_)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Specified collision mesh frame '"
                                    << collision_mesh_frame_
                                    << "' does not exist");
    return false;
  }

  scene_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>(
      "ik_planning_scene", 1);

  // Add the collision object to the planning scene
  const std::string object_name = "reach_object";
  moveit_msgs::msg::CollisionObject obj = utils::createCollisionObject(
      collision_mesh_package_, collision_mesh_frame_, object_name);
  if (!scene_->processCollisionObjectMsg(obj)) {
    RCLCPP_ERROR(LOGGER, "Failed to add collision mesh to planning scene");
    return false;
  } else {
    scene_->getAllowedCollisionMatrixNonConst().setEntry(object_name,
                                                         touch_links_, true);
  }

  RCLCPP_INFO_STREAM(LOGGER, "Successfully initialized MoveItIKSolver plugin");
  return true;
}

std::optional<double> MoveItIKSolver::solveIKFromSeed(
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
    fraction = 1.0;
    solution.clear();
    state.copyJointGroupPositions(jmg_, solution);

    // Convert back to map
    std::map<std::string, double> solution_map;
    for (std::size_t i = 0; i < solution.size(); ++i) {
      solution_map.emplace(joint_names[i], solution[i]);
    }

    return eval_->calculateScore(solution_map);
  } else {
    fraction = 0.0;
    return {};
  }
}

bool MoveItIKSolver::isIKSolutionValid(moveit::core::RobotState* state,
                                       const moveit::core::JointModelGroup* jmg,
                                       const double* ik_solution) const {
  state->setJointGroupPositions(jmg, ik_solution);
  state->update();

  const bool colliding =
      scene_->isStateColliding(*state, jmg->getName(), false);
  const bool too_close =
      (scene_->distanceToCollision(
           *state, scene_->getAllowedCollisionMatrix()) < distance_threshold_);

  // visualization part for debugging purposes only
  /*
    if (!colliding && !too_close){
        scene_->setCurrentState(*state);
        moveit_msgs::msg::PlanningScene scene_msg;
        scene_->getPlanningSceneMsg(scene_msg);
        scene_pub_->publish(scene_msg);
    }
    */
  return (!colliding && !too_close);
}

std::vector<std::string> MoveItIKSolver::getJointNames() const {
  return jmg_->getActiveJointModelNames();
}

}  // namespace ik
}  // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::ik::MoveItIKSolver,
                       reach::plugins::IKSolverBase)
