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
#include <reach_ros/evaluation/distance_penalty_moveit.h>
#include <reach_ros/utils.h>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene/planning_scene.h>
#include <reach_core/plugin_utils.h>
#include <yaml-cpp/yaml.h>

namespace reach_ros
{
namespace evaluation
{
DistancePenaltyMoveIt::DistancePenaltyMoveIt(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                                             const double dist_threshold, int exponent,
                                             std::string collision_mesh_filename, std::vector<std::string> touch_links)
  : model_(model)
  , jmg_(model_->getJointModelGroup(planning_group))
  , dist_threshold_(dist_threshold)
  , exponent_(exponent)
  , collision_mesh_filename_(collision_mesh_filename)
  , touch_links_(std::move(touch_links))
{
  if (!jmg_)
    throw std::runtime_error("Failed to get joint model group");

  scene_.reset(new planning_scene::PlanningScene(model_));

  // Add the collision mesh object to the planning scene
  const std::string object_name = "reach_object";
  moveit_msgs::CollisionObject obj =
      utils::createCollisionObject(collision_mesh_filename_, jmg_->getSolverInstance()->getBaseFrame(), object_name);
  if (!scene_->processCollisionObjectMsg(obj))
    throw std::runtime_error("Failed to add collision mesh to planning scene");

  scene_->getAllowedCollisionMatrixNonConst().setEntry(object_name, touch_links_, true);
}

double DistancePenaltyMoveIt::calculateScore(const std::map<std::string, double>& pose) const
{
  // Pull the joints from the planning group out of the input pose map
  std::vector<double> pose_subset = utils::transcribeInputMap(pose, jmg_->getActiveJointModelNames());
  moveit::core::RobotState state(model_);
  state.setJointGroupPositions(jmg_, pose_subset);
  state.update();

  const double dist = scene_->distanceToCollision(state, scene_->getAllowedCollisionMatrix());
  return std::pow((dist / dist_threshold_), exponent_);
}

reach::Evaluator::ConstPtr DistancePenaltyMoveItFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  auto dist_threshold = reach::get<double>(config, "distance_threshold");
  auto exponent = reach::get<int>(config, "exponent");
  auto collision_mesh_filename = reach::get<std::string>(config, "collision_mesh_filename");
  auto touch_links = reach::get<std::vector<std::string>>(config, "touch_links");

  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return std::make_shared<DistancePenaltyMoveIt>(model, planning_group, dist_threshold, exponent,
                                                 collision_mesh_filename, touch_links);
}

}  // namespace evaluation
}  // namespace reach_ros

EXPORT_IK_SOLVER_PLUGIN(reach_ros::evaluation::DistancePenaltyMoveItFactory, DistancePenaltyMoveIt)
