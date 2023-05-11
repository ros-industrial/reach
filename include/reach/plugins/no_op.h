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
#ifndef REACH_PLUGINS_NOOP_H
#define REACH_PLUGINS_NOOP_H

#include <reach/interfaces/evaluator.h>
#include <reach/interfaces/ik_solver.h>
#include <reach/interfaces/display.h>

#include <boost/shared_ptr.hpp>

namespace reach
{
struct NoOpEvaluator : public Evaluator
{
  double calculateScore(const std::map<std::string, double>&) const override;
};

struct NoOpEvaluatorFactory : public EvaluatorFactory
{
  virtual Evaluator::ConstPtr create(const YAML::Node&) const override;
};

struct NoOpIKSolver : public IKSolver
{
public:
  std::vector<std::string> getJointNames() const override;

  std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d&,
                                           const std::map<std::string, double>&) const override;
};

struct NoOpIKSolverFactory : public IKSolverFactory
{
  IKSolver::ConstPtr create(const YAML::Node&) const override;
};

struct NoOpDisplay : public Display
{
  void showEnvironment() const override;
  void updateRobotPose(const std::map<std::string, double>&) const override;
  void showReachNeighborhood(const std::map<std::size_t, ReachRecord>&) const override;
  void showResults(const ReachResult&) const override;
};

struct NoOpDisplayFactory : public DisplayFactory
{
  Display::ConstPtr create(const YAML::Node&) const override;
};

}  // namespace reach

#endif  // REACH_PLUGINS_NOOP_H
