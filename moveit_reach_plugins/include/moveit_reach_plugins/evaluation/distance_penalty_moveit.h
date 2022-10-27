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
#ifndef MOVEIT_REACH_PLUGINS_EVALUATION_DISTANCE_PENALTY_MOVEIT_H
#define MOVEIT_REACH_PLUGINS_EVALUATION_DISTANCE_PENALTY_MOVEIT_H

#include <reach_core/interfaces/evaluator.h>
#include <moveit_msgs/PlanningScene.h>

namespace moveit
{
namespace core
{
class RobotModel;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;
class JointModelGroup;
}  // namespace core
}  // namespace moveit

namespace planning_scene
{
class PlanningScene;
typedef std::shared_ptr<PlanningScene> PlanningScenePtr;
}  // namespace planning_scene

namespace moveit_reach_plugins
{
namespace evaluation
{
class DistancePenaltyMoveIt : public reach::Evaluator
{
public:
  DistancePenaltyMoveIt(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                        const double dist_threshold, int exponent, std::string collision_mesh_filename,
                        std::vector<std::string> touch_links);
  double calculateScore(const std::map<std::string, double>& pose) const override;

private:
  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* jmg_;
  const double dist_threshold_;
  const int exponent_;
  const std::string collision_mesh_filename_;
  const std::vector<std::string> touch_links_;

  planning_scene::PlanningScenePtr scene_;
};

struct DistancePenaltyMoveItFactory : public reach::EvaluatorFactory
{
  reach::Evaluator::ConstPtr create(const YAML::Node& config) const override;
};

}  // namespace evaluation
}  // namespace moveit_reach_plugins

#endif  // MOVEIT_REACH_PLUGINS_EVALUATION_DISTANCE_PENALTY_MOVEIT_H
