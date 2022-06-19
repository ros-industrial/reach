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
#ifndef REACH_CORE_STUDY_VISUALIZER_H
#define REACH_CORE_STUDY_VISUALIZER_H

#include <reach_core/reach_database.h>
#include <reach_core/plugins/reach_display_base.h>
#include <reach_core/plugins/ik_solver_base.h>
#include <reach_core/ik_helper.h>

namespace reach
{
namespace core
{
/**
 * @brief The ReachVisualizer class displays the results of the reach study and provides an interface for visualizing
 * the robot's work area from a given pose, and recalculating the IK solution at a given target pose
 */
class ReachVisualizer
{
public:
  using Ptr = boost::shared_ptr<ReachVisualizer>;

  /**
   * @brief ReachVisualizer
   * @param db
   * @param solver
   * @param display
   * @param neighbor_radius
   * @param search_tree
   */
  ReachVisualizer(ReachDatabase::Ptr db, reach::plugins::IKSolverBase::Ptr solver, reach::plugins::DisplayBase::Ptr display,
                  const double neighbor_radius, SearchTreePtr search_tree = nullptr);

  void update();

private:
  void reSolveIKCB(const std::string& fb);

  void showResultCB(const std::string& fb);

  void showSeedCB(const std::string& fb);

  void reachNeighborsDirectCB(const std::string& fb);

  void reachNeighborsRecursiveCB(const std::string& fb);

  ReachDatabase::Ptr db_;

  reach::plugins::IKSolverBase::Ptr solver_;

  reach::plugins::DisplayBase::Ptr display_;

  SearchTreePtr search_tree_;

  double neighbor_radius_;
};

}  // namespace core
}  // namespace reach

#endif  // REACH_CORE_STUDY_VISUALIZER_H
