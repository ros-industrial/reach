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
#include <moveit_reach_plugins/ik/moveit_ik_solver.h>
#include <moveit_reach_plugins/utils.h>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/PlanningScene.h>
#include <reach_core/plugin_utils.h>
#include <yaml-cpp/yaml.h>

namespace
{
template <typename T>
T clamp(const T& val, const T& low, const T& high)
{
  return std::max(low, std::min(val, high));
}

}  // namespace

namespace moveit_reach_plugins
{
namespace ik
{
MoveItIKSolver::MoveItIKSolver(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                               double dist_threshold, std::string collision_mesh_filename,
                               std::string collision_mesh_frame, std::vector<std::string> touch_links)
  : model_(model)
  , jmg_(model_->getJointModelGroup(planning_group))
  , distance_threshold_(dist_threshold)
  , collision_mesh_filename_(std::move(collision_mesh_filename))
  , collision_mesh_frame_(std::move(collision_mesh_frame))
  , touch_links_(std::move(touch_links))
{
  if (!jmg_)
    throw std::runtime_error("Failed to initialize joint model group for planning group '" + planning_group + "'");

  scene_.reset(new planning_scene::PlanningScene(model_));

  // Check that the input collision mesh frame exists
  if (!scene_->knowsFrameTransform(collision_mesh_frame_))
    throw std::runtime_error("Specified collision mesh frame '" + collision_mesh_frame_ + "' does not exist");

  // Add the collision object to the planning scene
  const std::string object_name = "reach_object";
  moveit_msgs::CollisionObject obj =
      utils::createCollisionObject(collision_mesh_filename_, collision_mesh_frame_, object_name);
  if (!scene_->processCollisionObjectMsg(obj))
    throw std::runtime_error("Failed to add collision mesh to planning scene");

  scene_->getAllowedCollisionMatrixNonConst().setEntry(object_name, touch_links_, true);
}

std::vector<std::vector<double>> MoveItIKSolver::solveIK(const Eigen::Isometry3d& target,
                                                         const std::map<std::string, double>& seed) const
{
  moveit::core::RobotState state(model_);

  const std::vector<std::string>& joint_names = jmg_->getActiveJointModelNames();

  std::vector<double> seed_subset = utils::transcribeInputMap(seed, joint_names);
  state.setJointGroupPositions(jmg_, seed_subset);
  state.update();

  if (state.setFromIK(jmg_, target, 0.0, boost::bind(&MoveItIKSolver::isIKSolutionValid, this, _1, _2, _3)))
  {
    std::vector<double> solution;
    state.copyJointGroupPositions(jmg_, solution);

    return { solution };
  }

  return {};
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

reach::IKSolver::ConstPtr MoveItIKSolverFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  auto dist_threshold = reach::get<double>(config, "distance_threshold");
  auto collision_mesh_filename = reach::get<std::string>(config, "collision_mesh_filename");
  auto collision_mesh_frame = reach::get<std::string>(config, "collision_mesh_frame");
  auto touch_links = reach::get<std::vector<std::string>>(config, "touch_links");

  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return std::make_shared<MoveItIKSolver>(model, planning_group, dist_threshold, collision_mesh_filename,
                                          collision_mesh_frame, touch_links);
}

DiscretizedMoveItIKSolver::DiscretizedMoveItIKSolver(moveit::core::RobotModelConstPtr model,
                                                     const std::string& planning_group, double dist_threshold,
                                                     std::string collision_mesh_filename,
                                                     std::string collision_mesh_frame,
                                                     std::vector<std::string> touch_links, double dt)
  : MoveItIKSolver(model, planning_group, dist_threshold, collision_mesh_filename, collision_mesh_frame, touch_links)
  , dt_(dt)
{
}

std::vector<std::vector<double>> DiscretizedMoveItIKSolver::solveIK(const Eigen::Isometry3d& target,
                                                                    const std::map<std::string, double>& seed) const
{
  // Calculate the number of discretizations necessary to achieve discretization angle
  const static int n_discretizations = int((2.0 * M_PI) / dt_);

  std::vector<std::vector<double>> solutions;
  solutions.reserve(n_discretizations);

  for (int i = 0; i < n_discretizations; ++i)
  {
    Eigen::Isometry3d discretized_target(target * Eigen::AngleAxisd(double(i) * dt_, Eigen::Vector3d::UnitZ()));
    std::vector<std::vector<double>> tmp_sols = MoveItIKSolver::solveIK(discretized_target, seed);

    if (!tmp_sols.empty())
      solutions.push_back(tmp_sols.front());
  }

  return solutions;
}

reach::IKSolver::ConstPtr DiscretizedMoveItIKSolverFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  auto dist_threshold = reach::get<double>(config, "distance_threshold");
  auto collision_mesh_filename = reach::get<std::string>(config, "collision_mesh_filename");
  auto collision_mesh_frame = reach::get<std::string>(config, "collision_mesh_frame");
  auto touch_links = reach::get<std::vector<std::string>>(config, "touch_links");

  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  auto dt = std::abs(reach::get<double>(config, "discretization_angle"));
  double clamped_dt = clamp<double>(dt, 0.0, M_PI);
  if (std::abs(dt - clamped_dt) > 1.0e-6)
  {
    std::cout << "Clamping discretization angle between 0 and pi; new value is " << clamped_dt;
  }
  dt = clamped_dt;

  return std::make_shared<DiscretizedMoveItIKSolver>(model, planning_group, dist_threshold, collision_mesh_filename,
                                                     collision_mesh_frame, touch_links, dt);
}

}  // namespace ik
}  // namespace moveit_reach_plugins

EXPORT_IK_SOLVER_PLUGIN(moveit_reach_plugins::ik::MoveItIKSolverFactory, MoveItIKSolver)
EXPORT_IK_SOLVER_PLUGIN(moveit_reach_plugins::ik::DiscretizedMoveItIKSolverFactory, DiscretizedMoveItIKSolverFactory)
