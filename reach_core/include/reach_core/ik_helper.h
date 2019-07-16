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
#ifndef REACH_CORE_IK_HELPER_H
#define REACH_CORE_IK_HELPER_H

#include <reach_core/reach_database.h>
#include <reach_core/study_parameters.h>
#include <reach_core/plugins/ik_solver_base.h>

#include <boost/optional.hpp>
#include <flann/flann.h>
#include <flann/algorithms/kdtree_single_index.h>

namespace reach
{
namespace core
{

struct NeighborReachResult
{
  std::vector<std::string> reached_pts;
  double joint_distance = 0;
};

typedef flann::KDTreeSingleIndex<flann::L2_3D<double>> SearchTree;
typedef std::shared_ptr<SearchTree> SearchTreePtr;

NeighborReachResult reachNeighborsDirect(std::shared_ptr<ReachDatabase> db,
                                         const reach_msgs::ReachRecord& rec,
                                         reach::plugins::IKSolverBasePtr solver,
                                         const double radius,
                                         SearchTreePtr search_tree = nullptr);

void reachNeighborsRecursive(std::shared_ptr<ReachDatabase> db,
                             const reach_msgs::ReachRecord& msg,
                             reach::plugins::IKSolverBasePtr solver,
                             const double radius,
                             NeighborReachResult result,
                             SearchTreePtr search_tree = nullptr);

} // namespace core
} // namespace reach

#endif // REACH_CORE_IK_HELPER_H
