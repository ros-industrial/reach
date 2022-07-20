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
#include "moveit_reach_plugins/evaluation/manipulability_moveit.h"
#include "moveit_reach_plugins/utils.h"
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_model/joint_model_group.h>

#include <numeric>
#include <xmlrpcpp/XmlRpcException.h>

namespace moveit_reach_plugins
{
namespace evaluation
{
ManipulabilityMoveIt::ManipulabilityMoveIt() : reach::plugins::EvaluationBase()
{
  jacobian_row_subset_.resize(6);
  std::iota(jacobian_row_subset_.begin(), jacobian_row_subset_.end(), 0);
}

bool ManipulabilityMoveIt::initialize(XmlRpc::XmlRpcValue& config)
{
  if (!config.hasMember("planning_group"))
  {
    ROS_ERROR("MoveIt Manipulability Evaluation Plugin is missing 'planning_group' parameter");
    return false;
  }

  std::string planning_group;
  try
  {
    planning_group = static_cast<std::string>(config["planning_group"]);
  }
  catch (const XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR_STREAM(ex.getMessage());
    return false;
  }

  if (config.hasMember("jacobian_row_subset") &&
      config["jacobian_row_subset"].getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    std::set<Eigen::Index> subset_rows;
    for (std::size_t i = 0; i < config["jacobian_row_subset"].size(); ++i)
    {
      int row = static_cast<int>(config["jacobian_row_subset"][i]);
      if (row < 0 || row >= 6)
      {
        ROS_ERROR_STREAM("Invalid Jacobian row subset index provided: " << row << ". Must be on interval [0, 6)");
        return false;
      }

      subset_rows.insert(row);
    }

    if (subset_rows.empty())
    {
      ROS_ERROR_STREAM("Jacobian row subset is empty");
      return false;
    }

    std::copy(subset_rows.begin(), subset_rows.end(), std::back_inserter(jacobian_row_subset_));
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
    ROS_ERROR("Failed to initialize joint model group pointer");
    return false;
  }

  return true;
}

double ManipulabilityMoveIt::calculateScore(const std::map<std::string, double>& pose)
{
  // Calculate manipulability of kinematic chain of input robot pose
  moveit::core::RobotState state(model_);

  // Take the subset of joints in the joint model group out of the input pose
  std::vector<double> pose_subset;
  if (!utils::transcribeInputMap(pose, jmg_->getActiveJointModelNames(), pose_subset))
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": failed to transcribe input pose map");
    return 0.0;
  }

  state.setJointGroupPositions(jmg_, pose_subset);
  state.update();

  // Get the Jacobian matrix
  Eigen::MatrixXd jacobian = state.getJacobian(jmg_);

  // Extract the partial jacobian
  if (jacobian_row_subset_.size() < 6)
  {
    Eigen::MatrixXd partial_jacobian(jacobian_row_subset_.size(), jacobian.cols());
    for (Eigen::Index i = 0; i < jacobian_row_subset_.size(); ++i)
    {
      partial_jacobian.row(i) = jacobian.row(jacobian_row_subset_[i]);
    }

    jacobian = partial_jacobian;
  }

  // Calculate manipulability by multiplying Jacobian matrix singular values together
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  Eigen::MatrixXd singular_values = svd.singularValues();
  return singular_values.array().prod();
}

}  // namespace evaluation
}  // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::evaluation::ManipulabilityMoveIt, reach::plugins::EvaluationBase)
