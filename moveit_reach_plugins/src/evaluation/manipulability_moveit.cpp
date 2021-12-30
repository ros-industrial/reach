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

namespace moveit_reach_plugins
{
namespace evaluation
{

ManipulabilityMoveIt::ManipulabilityMoveIt()
  : reach::plugins::EvaluationBase()
{

}

bool ManipulabilityMoveIt::initialize(std::string& name, rclcpp::Node::SharedPtr node)
{
    std::string planning_group;

    std::string param_prefix ("ik_solver_config.evaluation_plugin.moveit_reach_plugins/evaluation/ManipulabilityMoveIt.");
    if(!node->get_parameter(param_prefix+ "planning_group", planning_group))
  {
    RCLCPP_ERROR(LOGGER, "MoveIt Manipulability Evaluation Plugin is missing 'planning_group' parameter");
    return false;
  }

  RCLCPP_INFO(LOGGER, "Creating shared robot model in the node '%s' using parameter robot_description", node->get_name());
//  model_ = moveit::planning_interface::getSharedRobotModel(node, "robot_description");
  model_ = moveit::planning_interface::getSharedRobotModelLoader(node, "robot_description")->getModel();
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

  RCLCPP_INFO(LOGGER, "moveit_reach_plugins/evaluation/ManipulabilityMoveIt initialized successfully.");
  return true;
}

double ManipulabilityMoveIt::calculateScore(const std::map<std::string, double>& pose)
{
  // Calculate manipulability of kinematic chain of input robot pose
  moveit::core::RobotState state(model_);

  // Take the subset of joints in the joint model group out of the input pose
  std::vector<double> pose_subset;
  if(!utils::transcribeInputMap(pose, jmg_->getActiveJointModelNames(), pose_subset))
  {
    RCLCPP_ERROR_STREAM(LOGGER, __FUNCTION__ << ": failed to transcribe input pose map");
    return 0.0f;
  }

  state.setJointGroupPositions(jmg_, pose_subset);
  state.update();

  // Get the Jacobian matrix
  Eigen::MatrixXd jacobian = state.getJacobian(jmg_);

  // Calculate manipulability by multiplying Jacobian matrix singular values together
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  Eigen::MatrixXd singular_values = svd.singularValues();
  double m = 1.0;
  for(unsigned int i = 0; i < singular_values.rows(); ++i)
  {
    m *= singular_values(i, 0);
  }
  return m;
}

} // namespace evaluation
} // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::evaluation::ManipulabilityMoveIt, reach::plugins::EvaluationBase)
