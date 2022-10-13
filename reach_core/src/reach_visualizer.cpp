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
#include <reach_core/reach_visualizer.h>
#include <reach_core/utils.h>

namespace reach
{
ReachVisualizer::ReachVisualizer(ReachDatabase::Ptr db, IKSolver::ConstPtr solver, Evaluator::ConstPtr evaluator,
                                 Display::ConstPtr display, const double neighbor_radius)
  : db_(db)
  , solver_(solver)
  , evaluator_(evaluator)
  , display_(display)
  , search_tree_(createSearchTree(*db_))
  , neighbor_radius_(neighbor_radius)
{
  display_->showEnvironment();
}

void ReachVisualizer::reSolveIK(const std::string& marker_name)
{
  ReachRecord lookup = db_->get(marker_name);

  // Re-solve IK at the selected marker
  std::vector<double> goal_pose;
  double score;
  std::tie(goal_pose, score) = evaluateIK(lookup.goal, lookup.seed_state, solver_, evaluator_);

  lookup.reached = true;
  lookup.score = score;
  lookup.goal_state = zip(solver_->getJointNames(), goal_pose);

  // Update the interactive marker server
  display_->updateRobotPose(lookup.goal_state);

  // Update the database
  db_->put(lookup);
}

void ReachVisualizer::showResult(const std::string& marker_name) const
{
  ReachRecord lookup = db_->get(marker_name);
  display_->updateRobotPose(lookup.goal_state);
}

void ReachVisualizer::showSeed(const std::string& marker_name) const
{
  ReachRecord lookup = db_->get(marker_name);
  display_->updateRobotPose(lookup.seed_state);
}

void ReachVisualizer::reachNeighbors(const std::string& record_id, const bool recursive) const
{
  ReachRecord lookup = db_->get(record_id);
  std::vector<ReachRecord> result;
  if (recursive)
  {
    NeighborReachResult neighbors;
    reachNeighborsRecursive(db_, lookup, solver_, evaluator_, neighbor_radius_, neighbors, search_tree_);
    result.reserve(neighbors.reached_pts.size());
    std::transform(neighbors.reached_pts.begin(), neighbors.reached_pts.end(), std::back_inserter(result),
                   [this](const std::string& name) { return db_->get(name); });
  }
  else
  {
    result = reachNeighborsDirect(db_, lookup, solver_, evaluator_, neighbor_radius_, search_tree_);
  }

  display_->updateRobotPose(lookup.goal_state);
  display_->showReachNeighborhood(result);
}

}  // namespace reach
