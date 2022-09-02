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
double recurse(const moveit::core::JointModel* joint, const moveit::core::RobotState& state,
               const std::string& tcp_frame)
{
  const moveit::core::LinkModel* child_link = joint->getChildLinkModel();
  if (child_link->getName() == tcp_frame)
    return child_link->getJointOriginTransform().translation().norm();

  std::vector<const moveit::core::JointModel*> children = child_link->getChildJointModels();

  // Anything other than 1 child suggests that there has been a branch of joints
  if (children.size() != 1)
    throw std::runtime_error("The robot model urdf has branching joints after the last active joint. This "
                             "configuration is currently unsupported");

  switch (children.at(0)->getType())
  {
    case moveit::core::JointModel::FIXED:
      break;
    default:
      throw std::runtime_error("The robot model contains non-fixed joints after the last active joint. This "
                               "configuration is currently unsupported");
  }

  double d = child_link->getJointOriginTransform().translation().norm();
  d += recurse(children.at(0), state, tcp_frame);
  return d;
}

ManipulabilityMoveIt::ManipulabilityMoveIt() : reach::plugins::EvaluationBase()
{
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
  else
  {
    jacobian_row_subset_.resize(6);
    std::iota(jacobian_row_subset_.begin(), jacobian_row_subset_.end(), 0);
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

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  Eigen::MatrixXd singular_values = svd.singularValues();
  return calculateScore(singular_values);
}

double ManipulabilityMoveIt::calculateScore(const Eigen::MatrixXd& jacobian_singular_values)
{
  return jacobian_singular_values.array().prod();
}

double ManipulabilityRatio::calculateScore(const Eigen::MatrixXd& jacobian_singular_values)
{
  return jacobian_singular_values.minCoeff() / jacobian_singular_values.maxCoeff();
}

bool ManipulabilityNormalized::initialize(XmlRpc::XmlRpcValue& config)
{
  bool ret = ManipulabilityMoveIt::initialize(config);

  const std::string excluded_links_param = "excluded_links";
  if (config.hasMember(excluded_links_param) &&
      config[excluded_links_param].getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < config[excluded_links_param].size(); ++i)
    {
      std::string excluded_link = static_cast<std::string>(config[excluded_links_param][i]);
      excluded_links_.push_back(excluded_link);
    }

    if (excluded_links_.empty())
    {
      ROS_ERROR_STREAM("Exclusion list is empty");
      return false;
    }
  }

  calculateCharacteristicLength();
  return ret;
}

double ManipulabilityNormalized::calculateScore(const Eigen::MatrixXd& jacobian_singular_values)
{
  if (characteristic_length_ == 0)
    throw std::runtime_error("The model must have a nonzero characteristic length");

  return ManipulabilityMoveIt::calculateScore(jacobian_singular_values) / characteristic_length_;
}

double ManipulabilityNormalized::calculateCharacteristicLength()
{
  moveit::core::RobotState state(model_);

  std::vector<const moveit::core::JointModel*> active_joints = jmg_->getActiveJointModels();

  characteristic_length_ = 0.0;

  const std::string tcp_frame = jmg_->getSolverInstance()->getTipFrame();
  for (std::size_t i = 0; i < active_joints.size() - 1; ++i)
  {
    const moveit::core::JointModel* aj = active_joints.at(i);

    const moveit::core::LinkModel* child_link = aj->getChildLinkModel();
    std::string child_link_name = child_link->getName();

    // Skip this joint if its child link is in the exclusion list
    if (std::any_of(excluded_links_.begin(), excluded_links_.end(),
                    [&child_link_name](std::string excluded_link) { return (excluded_link == child_link_name); }))
      continue;

    // Calculate the transformation of fully extended primsatic joints
    if (aj->getType() == moveit::core::JointModel::PRISMATIC)
    {
      std::string joint_name = aj->getName();
      double max_position = aj->getVariableBounds().at(0).max_position_;
      state.setJointPositions(joint_name, &max_position);
    }

    Eigen::Isometry3d joint_transform = state.getJointTransform(aj);

    double joint_norm = joint_transform.translation().norm();
    characteristic_length_ += joint_norm;

    Eigen::Isometry3d transform = aj->getChildLinkModel()->getJointOriginTransform();
    double norm = transform.translation().norm();
    characteristic_length_ += norm;
  }

  // Recurse down the remaining tree of joints until we get to the TCP frame
  characteristic_length_ += recurse(active_joints.back(), state, tcp_frame);

  return characteristic_length_;
}

}  // namespace evaluation
}  // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::evaluation::ManipulabilityMoveIt, reach::plugins::EvaluationBase)
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::evaluation::ManipulabilityNormalized, reach::plugins::EvaluationBase)
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::evaluation::ManipulabilityRatio, reach::plugins::EvaluationBase)
