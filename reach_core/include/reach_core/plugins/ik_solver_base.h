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
#ifndef REACH_CORE_PLUGINS_IK_IK_SOLVER_BASE_H
#define REACH_CORE_PLUGINS_IK_IK_SOLVER_BASE_H

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model/robot_model.h>
#include <sensor_msgs/msg/joint_state.hpp>

namespace reach {
namespace plugins {

/**
 * @brief Base class solving IK at a given reach study location
 */
class IKSolverBase {
 public:
  IKSolverBase() {}

  virtual ~IKSolverBase() {}

  /**
   * @brief initialize
   * @param config
   * @return
   */
  virtual bool initialize(
      std::string &name, rclcpp::Node::SharedPtr node,
      const std::shared_ptr<const moveit::core::RobotModel> model) = 0;

  /**
   * @brief solveIKFromSeed attempts to find a valid IK solution for the given
   * target pose starting from the input seed state. If a solution is found, the
   * resulting IK solution is saved, and the pose is scored according to the
   * specified cost function plugin
   * @param seed
   * @param solution
   * @return a boost optional type indicating the success of the IK solution and
   * containing the score of the solution
   */
  virtual std::optional<double> solveIKFromSeed(
      const Eigen::Isometry3d &target,
      const std::map<std::string, double> &seed, std::vector<double> &solution,
      std::vector<double> &joint_space_trajectory,
      std::vector<double> &cartesian_space_waypoints, double &fraction) = 0;

  /**
   * @brief getJointNames
   * @return
   */
  virtual std::vector<std::string> getJointNames() const = 0;

 public:
  rclcpp::Node::SharedPtr node_;
};
typedef std::shared_ptr<IKSolverBase> IKSolverBasePtr;

}  // namespace plugins
}  // namespace reach

#endif  // REACH_CORE_PLUGINS_IK_IK_SOLVER_BASE_H
