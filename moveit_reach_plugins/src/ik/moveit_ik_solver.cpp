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
#include <moveit_msgs/PlanningScene.h>
#include <pluginlib/class_loader.h>
#include <xmlrpcpp/XmlRpcException.h>

namespace moveit_reach_plugins
{
namespace ik
{
const static std::string PACKAGE = "reach_core";
const static std::string EVAL_PLUGIN_BASE = "reach::plugins::EvaluationBase";

MoveItIKSolver::MoveItIKSolver() : reach::plugins::IKSolverBase(), class_loader_(PACKAGE, EVAL_PLUGIN_BASE)
{
}

bool MoveItIKSolver::initialize(XmlRpc::XmlRpcValue& config)
{
  if (!config.hasMember("planning_group") || !config.hasMember("distance_threshold") ||
      !config.hasMember("collision_mesh_filename") || !config.hasMember("collision_mesh_frame") ||
      !config.hasMember("touch_links") || !config.hasMember("evaluation_plugin"))
  {
    ROS_ERROR("MoveIt IK Solver Plugin is missing one or more configuration parameters");
    return false;
  }

  std::string planning_group;
  try
  {
    planning_group = std::string(config["planning_group"]);
    distance_threshold_ = double(config["distance_threshold"]);
    collision_mesh_filename_ = std::string(config["collision_mesh_filename"]);
    collision_mesh_frame_ = std::string(config["collision_mesh_frame"]);

    for (int i = 0; i < config["touch_links"].size(); ++i)
    {
      touch_links_.push_back(config["touch_links"][i]);
    }

    try
    {
      eval_ = class_loader_.createInstance(config["evaluation_plugin"]["name"]);
    }
    catch (const pluginlib::ClassLoaderException& ex)
    {
      ROS_ERROR_STREAM(ex.what());
    }

    if (!eval_->initialize(config["evaluation_plugin"]))
    {
      ROS_ERROR_STREAM("Failed to initialize evaluation plugin");
      return false;
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
    ROS_ERROR("Failed to initialize robot model pointer");
    return false;
  }

  jmg_ = model_->getJointModelGroup(planning_group);
  if (!jmg_)
  {
    ROS_ERROR_STREAM("Failed to get joint model group for '" << planning_group << "'");
    return false;
  }

  scene_.reset(new planning_scene::PlanningScene(model_));

  // Check that the input collision mesh frame exists
  if (!scene_->knowsFrameTransform(collision_mesh_frame_))
  {
    ROS_ERROR_STREAM("Specified collision mesh frame '" << collision_mesh_frame_ << "' does not exist");
    return false;
  }

  // Add the collision object to the planning scene
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

  ROS_INFO_STREAM("Successfully initialized MoveItIKSolver plugin");
  return true;
}

boost::optional<double> MoveItIKSolver::solveIKFromSeed(const Eigen::Isometry3d& target,
                                                        const std::map<std::string, double>& seed,
                                                        std::vector<double>& solution)
{
  moveit::core::RobotState state(model_);

  const std::vector<std::string>& joint_names = jmg_->getActiveJointModelNames();

  std::vector<double> seed_subset;
  if (!utils::transcribeInputMap(seed, joint_names, seed_subset))
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": failed to transcribe input pose map");
    return {};
  }

  state.setJointGroupPositions(jmg_, seed_subset);
  state.update();

  const static int SOLUTION_ATTEMPTS = 3;
  const static double SOLUTION_TIMEOUT = 0.2;

  if (state.setFromIK(jmg_, target, SOLUTION_ATTEMPTS, SOLUTION_TIMEOUT,
                      boost::bind(&MoveItIKSolver::isIKSolutionValid, this, _1, _2, _3)))
  {
    solution.clear();
    state.copyJointGroupPositions(jmg_, solution);

    // Convert back to map
    std::map<std::string, double> solution_map;
    for (std::size_t i = 0; i < solution.size(); ++i)
    {
      solution_map.emplace(joint_names[i], solution[i]);
    }

    return eval_->calculateScore(solution_map);
  }
  else
  {
    return {};
  }
}

bool MoveItIKSolver::isIKSolutionValid(moveit::core::RobotState* state, const moveit::core::JointModelGroup* jmg,
                                       const double* ik_solution) const
{
  state->setJointGroupPositions(jmg, ik_solution);
  state->update();

  const bool colliding = scene_->isStateColliding(*state, jmg->getName(), false);
  const bool too_close =
      (scene_->distanceToCollision(*state, scene_->getAllowedCollisionMatrix()) < distance_threshold_);

  return (!colliding && !too_close);
}

std::vector<std::string> MoveItIKSolver::getJointNames() const
{
  return jmg_->getActiveJointModelNames();
}

}  // namespace ik
}  // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::ik::MoveItIKSolver, reach::plugins::IKSolverBase)
