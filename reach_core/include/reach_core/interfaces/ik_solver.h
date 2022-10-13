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

#include <reach_core/interfaces/evaluator.h>

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <vector>

namespace YAML
{
class Node;
}

namespace reach
{
/**
 * @brief Base class solving IK at a given reach study location
 */
struct IKSolver
{
public:
  using Ptr = boost::shared_ptr<IKSolver>;
  using ConstPtr = boost::shared_ptr<const IKSolver>;

  IKSolver();
  virtual ~IKSolver() = default;

  virtual std::vector<std::string> getJointNames() const = 0;

  virtual std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d& target,
                                                   const std::map<std::string, double>& seed) const = 0;
};

struct IKSolverFactory
{
  using Ptr = boost::shared_ptr<IKSolverFactory>;
  using ConstPtr = boost::shared_ptr<const IKSolverFactory>;

  IKSolverFactory() = default;
  virtual ~IKSolverFactory() = default;

  virtual IKSolver::ConstPtr create(const YAML::Node& config) const = 0;
};

}  // namespace reach

#endif  // REACH_CORE_PLUGINS_IK_IK_SOLVER_BASE_H
