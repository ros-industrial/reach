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
#include "moveit_reach_plugins/evaluation/joint_penalty_moveit.h"
#include "moveit_reach_plugins/utils.h"
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/common_planning_interface_objects/common_objects.h>

namespace moveit_reach_plugins
{
namespace evaluation
{

JointPenaltyMoveIt::JointPenaltyMoveIt()
  : reach::plugins::EvaluationBase()
{

}

bool JointPenaltyMoveIt::initialize(std::string& name, rclcpp::Node::SharedPtr node)
{
    std::string planning_group;

    std::string param_prefix("ik_solver_config.evaluation_plugin.moveit_reach_plugins/evaluation/JointPenaltyMoveIt.");
    if(!node->get_parameter(param_prefix+ "planning_group", planning_group))
  {
    RCLCPP_ERROR(LOGGER, "MoveIt Joint Penalty Evaluation Plugin is missing 'planning_group' parameter");
    return false;
  }

  model_ = moveit::planning_interface::getSharedRobotModel(node, "robot_description");

  if(!model_)
  {
    RCLCPP_ERROR(LOGGER, "Failed to initialize robot model pointer");
    return false;
  }

  jmg_ = model_->getJointModelGroup(planning_group);
  if(!jmg_)
  {
    RCLCPP_ERROR(LOGGER, "Failed to initialize joint model group pointer");
    return false;
  }

  joint_limits_ = getJointLimits();

  return true;
}

double JointPenaltyMoveIt::calculateScore(const std::map<std::string, double>& pose)
{
  std::vector<double> max, min;
  min = joint_limits_[0];
  max = joint_limits_[1];

  // Pull the joints from the planning group out of the input pose map
  std::vector<double> pose_subset;
  if(!utils::transcribeInputMap(pose, jmg_->getActiveJointModelNames(), pose_subset))
  {
    RCLCPP_ERROR_STREAM(LOGGER, __FUNCTION__ << ": failed to transcribe input pose map");
    return 0.0f;
  }

  double penalty = 1.0;
  for(std::size_t i = 0; i < max.size(); ++i)
  {
    double range = max[i] - min[i];
    penalty *= ((pose_subset[i] - min[i])*(max[i] - pose_subset[i])) / std::pow(range, 2);
  }
  return std::max(0.0, 1.0 - std::exp(-1.0 * penalty));
}

std::vector<std::vector<double>> JointPenaltyMoveIt::getJointLimits()
{
  std::vector<double> max, min;
  // Get joint limits
  const auto limits_vec = jmg_->getActiveJointModelsBounds();
  for(std::size_t i = 0; i < limits_vec.size(); ++i)
  {
    const auto& bounds_vec = *limits_vec[i];
    if(bounds_vec.size() > 1)
    {
      RCLCPP_FATAL(LOGGER, "Joint has more than one DOF; can't pull joint limits correctly");
    }
    max.push_back(bounds_vec[0].max_position_);
    min.push_back(bounds_vec[0].min_position_);
  }
  std::vector<std::vector<double>> joint_limits;
  joint_limits.push_back(min);
  joint_limits.push_back(max);
  return joint_limits;
}

} // namespace evaluation
} // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::evaluation::JointPenaltyMoveIt, reach::plugins::EvaluationBase)
