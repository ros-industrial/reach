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
#include <reach_core/interfaces/display.h>
#include <reach_core/interfaces/ik_solver.h>
#include <reach_core/utils.h>

namespace reach
{
/**
 * @brief The ReachVisualizer class displays the results of the reach study and provides an interface for visualizing
 * the robot's work area from a given pose, and recalculating the IK solution at a given target pose
 */
class ReachVisualizer
{
public:
  using Ptr = boost::shared_ptr<ReachVisualizer>;

  ReachVisualizer(ReachDatabase::Ptr db, IKSolver::ConstPtr solver,
                  Display::ConstPtr display, const double neighbor_radius);

  void reSolveIK(const std::string& record_id);
  void showResult(const std::string& record_id) const;
  void showSeed(const std::string& record_id) const;
  void reachNeighbors(const std::string& record_id, const bool recursive = false);

protected:
  ReachDatabase::Ptr db_;
  IKSolver::ConstPtr solver_;
  Display::ConstPtr display_;
  SearchTreePtr search_tree_ = nullptr;
  double neighbor_radius_;
};

}  // namespace reach

#endif  // REACH_CORE_STUDY_VISUALIZER_H
