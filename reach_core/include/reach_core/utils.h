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

#include <atomic>
#include <map>
#include <memory>
#include <vector>
#include <Eigen/Dense>

// Forward declare flann classes
namespace flann
{
template <typename T>
class KDTreeSingleIndex;

template <typename T>
class L2_3D;
}

namespace reach
{
/**
 * @brief integerProgressPrinter
 * @param current_counter
 * @param previous_pct
 * @param total_size
 */
void integerProgressPrinter(std::atomic<int>& current_counter, std::atomic<int>& previous_pct, const int total_size);

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

struct NeighborReachResult
{
  std::vector<std::string> reached_pts;
  double joint_distance = 0;
};

typedef flann::KDTreeSingleIndex<flann::L2_3D<double>> SearchTree;
typedef std::shared_ptr<SearchTree> SearchTreePtr;

NeighborReachResult reachNeighborsDirect(ReachDatabase::Ptr db, const ReachRecord& rec,
                                         IKSolver::ConstPtr solver, const double radius,
                                         SearchTreePtr search_tree = nullptr);

void reachNeighborsRecursive(ReachDatabase::Ptr db, const ReachRecord& msg,
                             IKSolver::ConstPtr solver, const double radius,
                             NeighborReachResult result, SearchTreePtr search_tree = nullptr);

}  // namespace reach

#endif  // REACH_UTILS_GENERAL_UTILS_H
