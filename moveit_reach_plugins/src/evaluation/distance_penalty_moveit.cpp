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
#include "moveit_reach_plugins/evaluation/distance_penalty_moveit.h"
#include "moveit_reach_plugins/utils.h"
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene/planning_scene.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace moveit_reach_plugins
{
namespace evaluation
{

DistancePenaltyMoveIt::DistancePenaltyMoveIt()
  : reach::plugins::EvaluationBase()
{

}

bool DistancePenaltyMoveIt::initialize(std::string& name, rclcpp::Node::SharedPtr node)
{
    std::string planning_group;

    if(!node->get_parameter("ik_solver_config.evaluation_plugin.moveit_reach_plugins/evaluation/DistancePenaltyMoveIt.planning_group", planning_group) ||
     !node->get_parameter("ik_solver_config.evaluation_plugin.moveit_reach_plugins/evaluation/DistancePenaltyMoveIt.distance_threshold", dist_threshold_) ||
     !node->get_parameter("ik_solver_config.evaluation_plugin.moveit_reach_plugins/evaluation/DistancePenaltyMoveIt.collision_mesh_package", collision_mesh_package_) ||
     !node->get_parameter("ik_solver_config.evaluation_plugin.moveit_reach_plugins/evaluation/DistancePenaltyMoveIt.collision_mesh_filename_path", collision_mesh_filename_path_) ||
     !node->get_parameter("ik_solver_config.evaluation_plugin.moveit_reach_plugins/evaluation/DistancePenaltyMoveIt.collision_mesh_frame", collision_mesh_frame_) ||
     !node->get_parameter("ik_solver_config.evaluation_plugin.moveit_reach_plugins/evaluation/DistancePenaltyMoveIt.touch_links", touch_links_) ||
     !node->get_parameter("ik_solver_config.evaluation_plugin.moveit_reach_plugins/evaluation/DistancePenaltyMoveIt.exponent", exponent_))
  {
    RCLCPP_ERROR(LOGGER, "MoveIt Distance Penalty Evaluation plugin is missing one or more configuration parameters");
    return false;
  }

    if (std::find(touch_links_.begin(), touch_links_.end(), "") != touch_links_.end()){
        touch_links_.clear();
    }

    model_ = moveit::planning_interface::getSharedRobotModelLoader(node, "robot_description")->getModel();

  if(!model_)
  {
    RCLCPP_ERROR(LOGGER, "Failed to load robot model");
    return false;
  }

  jmg_ = model_->getJointModelGroup(planning_group);
  if(!jmg_)
  {
      RCLCPP_ERROR(LOGGER, "Failed to initialize joint model group pointer");
    return false;
  }

  scene_.reset(new planning_scene::PlanningScene (model_));

  // Check that the collision mesh frame exists
  if(!scene_->knowsFrameTransform(collision_mesh_frame_))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Specified collision mesh frame '" << collision_mesh_frame_ << "' does not exist");
    return false;
  }

  // Add the collision mesh object to the planning scene
  const std::string object_name = "reach_object";

    moveit_msgs::msg::CollisionObject obj = utils::createCollisionObject(collision_mesh_package_, collision_mesh_frame_, object_name);
  if(!scene_->processCollisionObjectMsg(obj))
  {
      RCLCPP_ERROR(LOGGER, "Failed to add collision mesh to planning scene");
    return false;
  }
  else
  {
    scene_->getAllowedCollisionMatrixNonConst().setEntry(object_name, touch_links_, true);
  }

  return true;
}

double DistancePenaltyMoveIt::calculateScore(const std::map<std::string, double>& pose)
{
  // Pull the joints from the planning group out of the input pose map
  std::vector<double> pose_subset;
  if(!utils::transcribeInputMap(pose, jmg_->getActiveJointModelNames(), pose_subset))
  {
    RCLCPP_ERROR_STREAM(LOGGER, __FUNCTION__ << ": failed to transcribe input pose map");
    return 0.0f;
  }

  moveit::core::RobotState state (model_);
  state.setJointGroupPositions(jmg_, pose_subset);
  state.update();

  const double dist = scene_->distanceToCollision(state, scene_->getAllowedCollisionMatrix());
  return std::pow((dist / dist_threshold_), exponent_);
}

} // namespace evaluation
} // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::evaluation::DistancePenaltyMoveIt, reach::plugins::EvaluationBase)
