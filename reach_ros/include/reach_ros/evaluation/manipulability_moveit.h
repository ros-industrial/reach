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
#ifndef REACH_ROS_EVALUATION_MANIPULABILITY_EVALUATION_H
#define REACH_ROS_EVALUATION_MANIPULABILITY_EVALUATION_H

#include <Eigen/Dense>

#include <reach/interfaces/evaluator.h>

namespace moveit
{
namespace core
{
class RobotModel;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;
class JointModelGroup;
}  // namespace core
}  // namespace moveit

namespace reach_ros
{
namespace evaluation
{
class ManipulabilityMoveIt : public reach::Evaluator
{
public:
  ManipulabilityMoveIt(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                       std::vector<Eigen::Index> jacobian_row_subset);
  double calculateScore(const std::map<std::string, double>& pose) const override;

protected:
  virtual double calculateScore(const Eigen::MatrixXd& jacobian_singular_values) const;

  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* jmg_;
  const std::vector<Eigen::Index> jacobian_row_subset_;
};

struct ManipulabilityMoveItFactory : public reach::EvaluatorFactory
{
  using reach::EvaluatorFactory::EvaluatorFactory;

  reach::Evaluator::ConstPtr create(const YAML::Node& config) const override;
};

/** @brief Computes the manipulability of a robot pose divided by the characteristic length of the robot */
class ManipulabilityScaled : public ManipulabilityMoveIt
{
public:
  ManipulabilityScaled(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                       std::vector<Eigen::Index> jacobian_row_subset, std::vector<std::string> excluded_links);
  virtual double calculateScore(const Eigen::MatrixXd& jacobian_singular_values) const override;

protected:
  const std::vector<std::string> excluded_links_;
  double characteristic_length_;
};

struct ManipulabilityScaledFactory : public reach::EvaluatorFactory
{
  using reach::EvaluatorFactory::EvaluatorFactory;

  reach::Evaluator::ConstPtr create(const YAML::Node& config) const override;
};

class ManipulabilityRatio : public ManipulabilityMoveIt
{
public:
  using ManipulabilityMoveIt::ManipulabilityMoveIt;
  virtual double calculateScore(const Eigen::MatrixXd& jacobian_singular_values) const override;
};

struct ManipulabilityRatioFactory : public reach::EvaluatorFactory
{
  using reach::EvaluatorFactory::EvaluatorFactory;

  reach::Evaluator::ConstPtr create(const YAML::Node& config) const override;
};

/**
 * @brief Computes the characteristic length of the joint model group by walking from the base link of the group to the
 * tip link and summing distances between links
 */
double calculateCharacteristicLength(moveit::core::RobotModelConstPtr model, const moveit::core::JointModelGroup* jmg,
                                     const std::vector<std::string>& excluded_links);

}  // namespace evaluation
}  // namespace reach_ros

#endif  // REACH_ROS_EVALUATION_MANIPULABILITY_EVALUATION_H
