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

#include <reach_core/plugins/evaluation_base.h>

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <pluginlib/class_loader.h>
#include <vector>
#include <xmlrpcpp/XmlRpcValue.h>

namespace reach
{
namespace plugins
{
/**
 * @brief Base class solving IK at a given reach study location
 */
class IKSolverBase
{
public:
  using Ptr = boost::shared_ptr<IKSolverBase>;

  IKSolverBase();
  virtual ~IKSolverBase() = default;

  /**
   * @brief initialize
   * @param config
   * @return
   */
  void initialize(XmlRpc::XmlRpcValue& config);

  /**
   * @brief solveIKFromSeed attempts to find a valid IK solution for the given target pose starting from the input seed
   * state. If a solution is found, the resulting IK solution is saved, and the pose is scored according to the
   * specified cost function plugin
   * @param seed
   * @param solution
   * @return a boost optional type indicating the success of the IK solution and containing the score of the solution
   */
  std::tuple<std::vector<double>, double> solveIKFromSeed(const Eigen::Isometry3d& target,
                                                          const std::map<std::string, double>& seed) const;

  /**
   * @brief getJointNames
   * @return
   */
  virtual std::vector<std::string> getJointNames() const = 0;

protected:
  virtual void initializeImpl(const XmlRpc::XmlRpcValue& config) = 0;
  virtual std::vector<std::vector<double>> solveIKFromSeedImpl(const Eigen::Isometry3d& target,
                                                               const std::map<std::string, double>& seed) const = 0;

private:
  pluginlib::ClassLoader<EvaluationBase> loader_;
  EvaluationBase::Ptr eval_;
};

}  // namespace plugins
}  // namespace reach

#endif  // REACH_CORE_PLUGINS_IK_IK_SOLVER_BASE_H
