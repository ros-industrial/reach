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
#include <xmlrpcpp/XmlRpcException.h>

namespace moveit_reach_plugins
{
namespace evaluation
{
DistancePenaltyMoveIt::DistancePenaltyMoveIt() : reach::plugins::EvaluationBase()
{
}

bool DistancePenaltyMoveIt::initialize(XmlRpc::XmlRpcValue& config)
{
  if (!config.hasMember("planning_group") || !config.hasMember("distance_threshold") ||
      !config.hasMember("collision_mesh_filename") || !config.hasMember("collision_mesh_frame") ||
      !config.hasMember("touch_links") || !config.hasMember("exponent"))
  {
    ROS_ERROR("MoveIt Distance Penalty Evaluation plugin is missing one or more configuration parameters");
    return false;
  }

  std::string planning_group;
  try
  {
    planning_group = std::string(config["planning_group"]);
    dist_threshold_ = double(config["distance_threshold"]);
    exponent_ = int(config["exponent"]);
    collision_mesh_filename_ = std::string(config["collision_mesh_filename"]);
    collision_mesh_frame_ = std::string(config["collision_mesh_frame"]);
    for (int i = 0; i < config["touch_links"].size(); ++i)
    {
      touch_links_.push_back(config["touch_links"][i]);
    }
  }
  catch (const XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR_STREAM(ex.getMessage());
    return false;
  }

  model_ = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model_)
  {
    ROS_ERROR("Failed to load robot model");
    return false;
  }

  jmg_ = model_->getJointModelGroup(planning_group);
  if (!jmg_)
  {
    ROS_ERROR("Failed to initialize joint model group pointer");
    return false;
  }

  scene_.reset(new planning_scene::PlanningScene(model_));

  // Check that the collision mesh frame exists
  if (!scene_->knowsFrameTransform(collision_mesh_frame_))
  {
    ROS_ERROR_STREAM("Specified collision mesh frame '" << collision_mesh_frame_ << "' does not exist");
    return false;
  }

  // Add the collision mesh object to the planning scene
  const std::string object_name = "reach_object";
  moveit_msgs::CollisionObject obj =
      utils::createCollisionObject(collision_mesh_filename_, collision_mesh_frame_, object_name);
  if (!scene_->processCollisionObjectMsg(obj))
  {
    ROS_ERROR("Failed to add collision mesh to planning scene");
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
  if (!utils::transcribeInputMap(pose, jmg_->getActiveJointModelNames(), pose_subset))
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": failed to transcribe input pose map");
    return 0.0f;
  }

  moveit::core::RobotState state(model_);
  state.setJointGroupPositions(jmg_, pose_subset);
  state.update();

  const double dist = scene_->distanceToCollision(state, scene_->getAllowedCollisionMatrix());
  return std::pow((dist / dist_threshold_), exponent_);
}

}  // namespace evaluation
}  // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::evaluation::DistancePenaltyMoveIt, reach::plugins::EvaluationBase)
