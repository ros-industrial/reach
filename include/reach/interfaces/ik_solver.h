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
#ifndef reach_INTERFACES_IK_SOLVER_H
#define reach_INTERFACES_IK_SOLVER_H

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <vector>

namespace YAML
{
class Node;
}

#ifdef BUILD_PYTHON
namespace boost
{
namespace python
{
namespace numpy
{
class ndarray;
}
class list;
class dict;
}  // namespace python
}  // namespace boost
#endif

namespace reach
{
/**
 * @brief Interface for solving inverse kinematics
 */
struct IKSolver
{
public:
  using Ptr = std::shared_ptr<IKSolver>;
  using ConstPtr = std::shared_ptr<const IKSolver>;

  IKSolver() = default;
  virtual ~IKSolver() = default;

  /** @brief Returns the joint names of the robot */
  virtual std::vector<std::string> getJointNames() const = 0;

  /** @brief Solves IK for a given target pose and seed state */
  virtual std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d& target,
                                                   const std::map<std::string, double>& seed) const = 0;

#ifdef BUILD_PYTHON
  boost::python::list solveIK(const boost::python::numpy::ndarray& target, const boost::python::dict& seed) const;
#endif
};

/** @brief Plugin interface for generating IK solver interfaces */
struct IKSolverFactory
{
  using Ptr = std::shared_ptr<IKSolverFactory>;
  using ConstPtr = std::shared_ptr<const IKSolverFactory>;

  IKSolverFactory() = default;
  virtual ~IKSolverFactory() = default;

  virtual IKSolver::ConstPtr create(const YAML::Node& config) const = 0;

  static std::string getSection()
  {
    return IK_SOLVER_SECTION;
  }

#ifdef BUILD_PYTHON
  IKSolver::ConstPtr create(const boost::python::dict& pyyaml_config) const;
#endif
};

}  // namespace reach

#endif  // reach_INTERFACES_IK_SOLVER_H
