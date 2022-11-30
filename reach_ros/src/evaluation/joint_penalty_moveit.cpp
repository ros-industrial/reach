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
#include <reach_ros/evaluation/joint_penalty_moveit.h>
#include <reach_ros/utils.h>

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <reach/plugin_utils.h>
#include <yaml-cpp/yaml.h>

namespace reach_ros
{
namespace evaluation
{
JointPenaltyMoveIt::JointPenaltyMoveIt(moveit::core::RobotModelConstPtr model, const std::string& planning_group)
  : model_(model), jmg_(model_->getJointModelGroup(planning_group))
{
  if (!jmg_)
    throw std::runtime_error("Failed to get joint model group");

  std::tie(joints_min_, joints_max_) = getJointLimits();
}

double JointPenaltyMoveIt::calculateScore(const std::map<std::string, double>& pose) const
{
  // Pull the joints from the planning group out of the input pose map
  std::vector<double> pose_subset = utils::transcribeInputMap(pose, jmg_->getActiveJointModelNames());
  Eigen::Map<const Eigen::ArrayXd> min(joints_min_.data(), joints_min_.size());
  Eigen::Map<const Eigen::ArrayXd> max(joints_max_.data(), joints_max_.size());
  Eigen::Map<const Eigen::ArrayXd> joints(pose_subset.data(), pose_subset.size());

  Eigen::VectorXd score = 4 * ((joints - min) * (max - joints)) / (max - min).pow(2);
  return score.mean();
}

std::tuple<std::vector<double>, std::vector<double>> JointPenaltyMoveIt::getJointLimits()
{
  std::vector<double> max, min;
  // Get joint limits
  const auto limits_vec = jmg_->getActiveJointModelsBounds();
  for (std::size_t i = 0; i < limits_vec.size(); ++i)
  {
    const auto& bounds_vec = *limits_vec[i];
    if (bounds_vec.size() > 1)
      throw std::runtime_error("Joint has more than one DOF; can't pull joint limits correctly");

    max.push_back(bounds_vec[0].max_position_);
    min.push_back(bounds_vec[0].min_position_);
  }
  std::vector<std::vector<double>> joint_limits;
  joint_limits.push_back(min);
  joint_limits.push_back(max);
  return std::make_tuple(min, max);
}

reach::Evaluator::ConstPtr JointPenaltyMoveItFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");

  utils::initROS();
  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return std::make_shared<JointPenaltyMoveIt>(model, planning_group);
}

}  // namespace evaluation
}  // namespace reach_ros

EXPORT_EVALUATOR_PLUGIN(reach_ros::evaluation::JointPenaltyMoveItFactory, JointPenaltyMoveIt)
