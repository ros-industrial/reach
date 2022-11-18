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
#ifndef REACH_UTILS_GENERAL_UTILS_H
#define REACH_UTILS_GENERAL_UTILS_H

#include <reach_core/reach_database.h>
#include <reach_core/interfaces/ik_solver.h>
#include <reach_core/interfaces/evaluator.h>

#include <atomic>
#include <Eigen/Dense>
#include <map>
#include <memory>
#include <pcl/search/kdtree.h>
#include <vector>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/exceptions.h>

namespace reach
{
template <typename T>
static std::map<std::string, T> zip(const std::vector<std::string>& keys, const std::vector<T>& values)
{
  std::map<std::string, T> map;
  for (std::size_t i = 0; i < keys.size(); ++i)
  {
    map[keys[i]] = values.at(i);
  }
  return map;
}

std::tuple<std::vector<double>, double> evaluateIK(const Eigen::Isometry3d& target,
                                                   const std::map<std::string, double>& seed,
                                                   IKSolver::ConstPtr ik_solver, Evaluator::ConstPtr evaluator);

using SearchTreePtr = pcl::search::KdTree<pcl::PointXYZ>::Ptr;
SearchTreePtr createSearchTree(const ReachDatabase& db);

std::vector<ReachRecord> reachNeighborsDirect(ReachDatabase::ConstPtr db, const ReachRecord& rec,
                                              IKSolver::ConstPtr solver, Evaluator::ConstPtr evaluator,
                                              const double radius, SearchTreePtr search_tree = nullptr);

struct NeighborReachResult
{
  std::vector<std::string> reached_pts;
  double joint_distance = 0;
};

void reachNeighborsRecursive(ReachDatabase::ConstPtr db, const ReachRecord& msg, IKSolver::ConstPtr solver,
                             Evaluator::ConstPtr evaluator, const double radius, NeighborReachResult& result,
                             SearchTreePtr search_tree = nullptr);

}  // namespace reach

#endif  // REACH_UTILS_GENERAL_UTILS_H
