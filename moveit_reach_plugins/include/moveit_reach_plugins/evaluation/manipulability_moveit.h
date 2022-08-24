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
#ifndef MOVEIT_REACH_PLUGINS_EVALUATION_MANIPULABILITY_EVALUATION_H
#define MOVEIT_REACH_PLUGINS_EVALUATION_MANIPULABILITY_EVALUATION_H

#include <Eigen/Dense>

#include <reach_core/plugins/evaluation_base.h>

namespace moveit
{
namespace core
{
class RobotModel;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;
class JointModelGroup;
}  // namespace core
}  // namespace moveit

namespace moveit_reach_plugins
{
namespace evaluation
{
class ManipulabilityMoveIt : public reach::plugins::EvaluationBase
{
public:
  ManipulabilityMoveIt();

  virtual bool initialize(XmlRpc::XmlRpcValue& config) override;

  virtual double calculateScore(const std::map<std::string, double>& pose) override;

protected:
  virtual double calculateScore(const Eigen::MatrixXd& jacobian_singular_values);

  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* jmg_;
  std::vector<int> jacobian_row_subset_;
};

class ManipulabilityNormalized : public ManipulabilityMoveIt
{
public:
  using ManipulabilityMoveIt::ManipulabilityMoveIt;
  virtual bool initialize(XmlRpc::XmlRpcValue& config) override;

  virtual double calculateScore(const Eigen::MatrixXd& jacobian_singular_values) override;
  double calculateCharacteristicLength();

protected:
  std::vector<std::string> exclusion_list_;
  double characteristic_length_;
};

class ManipulabilityRatio : public ManipulabilityMoveIt
{
public:
  using ManipulabilityMoveIt::ManipulabilityMoveIt;
  virtual double calculateScore(const Eigen::MatrixXd& jacobian_singular_values) override;
};

}  // namespace evaluation
}  // namespace moveit_reach_plugins

#endif  // MOVEIT_REACH_PLUGINS_EVALUATION_MANIPULABILITY_EVALUATION_H
