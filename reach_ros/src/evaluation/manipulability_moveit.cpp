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
#include <reach_ros/evaluation/manipulability_moveit.h>
#include <reach_ros/utils.h>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_model/joint_model_group.h>
#include <numeric>
#include <reach/plugin_utils.h>
#include <yaml-cpp/yaml.h>

static std::vector<Eigen::Index> getJacobianRowSubset(const YAML::Node& config, const std::string& key = "jacobian_row_"
                                                                                                         "subset")
{
  std::vector<Eigen::Index> jacobian_row_subset;

  const YAML::Node& jrs_config = config[key];
  if (jrs_config.IsDefined())
  {
    std::set<Eigen::Index> subset_rows;
    for (auto it = jrs_config.begin(); it != jrs_config.end(); ++it)
    {
      int row = (*it).as<Eigen::Index>();
      if (row < 0 || row >= 6)
      {
        std::stringstream ss;
        ss << "Invalid Jacobian row subset index provided: " << row << ". Must be on interval [0, 6)";
        throw std::runtime_error(ss.str());
      }

      subset_rows.insert(row);
    }

    if (subset_rows.empty())
      throw std::runtime_error("Jacobian row subset is empty");

    std::copy(subset_rows.begin(), subset_rows.end(), std::back_inserter(jacobian_row_subset));
  }
  else
  {
    jacobian_row_subset.resize(6);
    std::iota(jacobian_row_subset.begin(), jacobian_row_subset.end(), 0);
  }

  return jacobian_row_subset;
}

static std::vector<std::string> getExcludedLinks(const YAML::Node& config, const std::string& key = "excluded_links")
{
  try
  {
    return reach::get<std::vector<std::string>>(config, key);
  }
  catch (const std::exception& ex)
  {
    return {};
  }
}

namespace reach_ros
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

ManipulabilityMoveIt::ManipulabilityMoveIt(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                                           std::vector<Eigen::Index> jacobian_row_subset)
  : model_(std::move(model))
  , jmg_(model_->getJointModelGroup(planning_group))
  , jacobian_row_subset_(std::move(jacobian_row_subset))
{
  if (!jmg_)
    throw std::runtime_error("Failed to initialize joint model group pointer");
}

double ManipulabilityMoveIt::calculateScore(const std::map<std::string, double>& pose) const
{
  // Calculate manipulability of kinematic chain of input robot pose
  moveit::core::RobotState state(model_);

  // Take the subset of joints in the joint model group out of the input pose
  std::vector<double> pose_subset = utils::transcribeInputMap(pose, jmg_->getActiveJointModelNames());
  state.setJointGroupPositions(jmg_, pose_subset);
  state.update();

  // Get the Jacobian matrix
  Eigen::MatrixXd jacobian = state.getJacobian(jmg_);

  // Extract the partial jacobian
  if (jacobian_row_subset_.size() < 6)
  {
    Eigen::MatrixXd partial_jacobian(jacobian_row_subset_.size(), jacobian.cols());
    for (std::size_t i = 0; i < jacobian_row_subset_.size(); ++i)
    {
      partial_jacobian.row(i) = jacobian.row(jacobian_row_subset_[i]);
    }

    jacobian = partial_jacobian;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  Eigen::MatrixXd singular_values = svd.singularValues();
  return calculateScore(singular_values);
}

double ManipulabilityMoveIt::calculateScore(const Eigen::MatrixXd& jacobian_singular_values) const
{
  return jacobian_singular_values.array().prod();
}

reach::Evaluator::ConstPtr ManipulabilityMoveItFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  std::vector<Eigen::Index> jacobian_row_subset = getJacobianRowSubset(config);

  utils::initROS();
  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel(reach_ros::utils::node, "robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return std::make_shared<ManipulabilityMoveIt>(model, planning_group, jacobian_row_subset);
}

double ManipulabilityRatio::calculateScore(const Eigen::MatrixXd& jacobian_singular_values) const
{
  return jacobian_singular_values.minCoeff() / jacobian_singular_values.maxCoeff();
}

reach::Evaluator::ConstPtr ManipulabilityRatioFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  std::vector<Eigen::Index> jacobian_row_subset = getJacobianRowSubset(config);

  utils::initROS();
  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel(reach_ros::utils::node, "robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return std::make_shared<ManipulabilityRatio>(model, planning_group, jacobian_row_subset);
}

ManipulabilityScaled::ManipulabilityScaled(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                                           std::vector<Eigen::Index> jacobian_row_subset,
                                           std::vector<std::string> excluded_links)
  : ManipulabilityMoveIt(model, planning_group, jacobian_row_subset), excluded_links_(std::move(excluded_links))
{
  characteristic_length_ = calculateCharacteristicLength(model_, jmg_, excluded_links_);
}

double ManipulabilityScaled::calculateScore(const Eigen::MatrixXd& jacobian_singular_values) const
{
  if (std::abs(characteristic_length_) < std::numeric_limits<double>::epsilon())
    throw std::runtime_error("The model must have a non-zero characteristic length");

  return ManipulabilityMoveIt::calculateScore(jacobian_singular_values) / characteristic_length_;
}

reach::Evaluator::ConstPtr ManipulabilityScaledFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  std::vector<Eigen::Index> jacobian_row_subset = getJacobianRowSubset(config);
  std::vector<std::string> excluded_links = getExcludedLinks(config);

  utils::initROS();
  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel(reach_ros::utils::node, "robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return std::make_shared<ManipulabilityScaled>(model, planning_group, jacobian_row_subset, excluded_links);
}

double calculateCharacteristicLength(moveit::core::RobotModelConstPtr model, const moveit::core::JointModelGroup* jmg,
                                     const std::vector<std::string>& excluded_links)
{
  moveit::core::RobotState state(model);

  std::vector<const moveit::core::JointModel*> active_joints = jmg->getActiveJointModels();

  double characteristic_length = 0.0;

  const std::string tcp_frame = jmg->getSolverInstance()->getTipFrame();
  for (std::size_t i = 0; i < active_joints.size() - 1; ++i)
  {
    const moveit::core::JointModel* aj = active_joints.at(i);

    const moveit::core::LinkModel* child_link = aj->getChildLinkModel();
    std::string child_link_name = child_link->getName();

    // Skip this joint if its child link is in the exclusion list
    if (std::any_of(excluded_links.begin(), excluded_links.end(),
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
    characteristic_length += joint_norm;

    Eigen::Isometry3d transform = aj->getChildLinkModel()->getJointOriginTransform();
    double norm = transform.translation().norm();
    characteristic_length += norm;
  }

  // Recurse down the remaining tree of joints until we get to the TCP frame
  characteristic_length += recurse(active_joints.back(), state, tcp_frame);

  return characteristic_length;
}

}  // namespace evaluation
}  // namespace reach_ros

EXPORT_EVALUATOR_PLUGIN(reach_ros::evaluation::ManipulabilityMoveItFactory, ManipulabilityMoveIt)
EXPORT_EVALUATOR_PLUGIN(reach_ros::evaluation::ManipulabilityScaledFactory, ManipulabilityScaledMoveIt)
EXPORT_EVALUATOR_PLUGIN(reach_ros::evaluation::ManipulabilityRatioFactory, ManipulabilityRatioMoveIt)
