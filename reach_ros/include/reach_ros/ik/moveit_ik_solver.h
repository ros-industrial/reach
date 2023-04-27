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
#ifndef REACH_ROS_IK_MOVEIT_IK_SOLVER_H
#define REACH_ROS_IK_MOVEIT_IK_SOLVER_H

#include <reach/interfaces/ik_solver.h>
#include <rclcpp/publisher.hpp>
#include <vector>
#include <moveit_msgs/msg/planning_scene.hpp>
 #include <functional>
 
namespace moveit
{
namespace core
{
class RobotModel;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;
class JointModelGroup;
class RobotState;
}  // namespace core
}  // namespace moveit

namespace planning_scene
{
class PlanningScene;
typedef std::shared_ptr<PlanningScene> PlanningScenePtr;
}  // namespace planning_scene

namespace reach_ros
{
namespace ik
{
class MoveItIKSolver : public reach::IKSolver
{
public:
  MoveItIKSolver(moveit::core::RobotModelConstPtr model, const std::string& planning_group, double dist_threshold);

  std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d& target,
                                           const std::map<std::string, double>& seed) const override;

  std::vector<std::string> getJointNames() const override;

  void setTouchLinks(const std::vector<std::string>& touch_links);
  void addCollisionMesh(const std::string& collision_mesh_filename, const std::string& collision_mesh_frame);
  std::string getKinematicBaseFrame() const;

protected:
  bool isIKSolutionValid(moveit::core::RobotState* state, const moveit::core::JointModelGroup* jmg,
                         const double* ik_solution) const;

  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* jmg_;
  const double distance_threshold_;

  planning_scene::PlanningScenePtr scene_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;

  static std::string COLLISION_OBJECT_NAME;
};

struct MoveItIKSolverFactory : public reach::IKSolverFactory
{
  reach::IKSolver::ConstPtr create(const YAML::Node& config) const override;
};

class DiscretizedMoveItIKSolver : public MoveItIKSolver
{
public:
  DiscretizedMoveItIKSolver(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                            double dist_threshold, double dt);

  std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d& target,
                                           const std::map<std::string, double>& seed) const override;

protected:
  const double dt_;
};

struct DiscretizedMoveItIKSolverFactory : public reach::IKSolverFactory
{
  reach::IKSolver::ConstPtr create(const YAML::Node& config) const override;
};

}  // namespace ik
}  // namespace reach_ros

#endif  // REACH_ROS_IK_MOVEIT_IK_SOLVER_H
