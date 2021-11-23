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
 * @brief The ReachVisualizer class displays the results of the reach study and provides an interface for visualizing the robot's work
 * area from a given pose, and recalculating the IK solution at a given target pose
 */
    class ReachVisualizer
    {
    public:
      /**
   * @brief ReachVisualizer
   * @param db
   * @param solver
   * @param display
   * @param neighbor_radius
   * @param search_tree
   */
      ReachVisualizer(ReachDatabasePtr db,
                      reach::plugins::IKSolverBasePtr solver,
                      reach::plugins::DisplayBasePtr display,
                      const double neighbor_radius,
                      SearchTreePtr search_tree = nullptr);

      void update();

    private:
      void reSolveIKCB(const visualization_msgs::msg::InteractiveMarkerFeedback *&fb);

      void showResultCB(const visualization_msgs::msg::InteractiveMarkerFeedback *&fb);

      void showSeedCB(const visualization_msgs::msg::InteractiveMarkerFeedback *&fb);

      void reachNeighborsDirectCB(const visualization_msgs::msg::InteractiveMarkerFeedback *&fb);

      void reachNeighborsRecursiveCB(const visualization_msgs::msg::InteractiveMarkerFeedback *&fb);

      ReachDatabasePtr db_;

      reach::plugins::IKSolverBasePtr solver_;

      reach::plugins::DisplayBasePtr display_;

      SearchTreePtr search_tree_;

      double neighbor_radius_;
    };
    typedef std::shared_ptr<ReachVisualizer> ReachVisualizerPtr;

  } // namespace core
} // namespace reach

#endif // REACH_CORE_STUDY_VISUALIZER_H
