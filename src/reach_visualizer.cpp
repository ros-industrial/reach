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
#include <reach/reach_visualizer.h>
#include <reach/utils.h>

namespace reach
{
static SearchTreePtr createSearchTree(const ReachResult& result)
{
  VectorIsometry3d poses;
  poses.reserve(result.size());
  std::transform(result.begin(), result.end(), std::back_inserter(poses), [](const ReachRecord& r) { return r.goal; });
  return createSearchTree(poses);
}

ReachVisualizer::ReachVisualizer(ReachResult result, IKSolver::ConstPtr solver, Evaluator::ConstPtr evaluator,
                                 Display::ConstPtr display, const double neighbor_radius)
  : result_(std::move(result))
  , solver_(solver)
  , evaluator_(evaluator)
  , display_(display)
  , search_tree_(createSearchTree(result_))
  , neighbor_radius_(neighbor_radius)
{
  display_->showEnvironment();
}

void ReachVisualizer::reSolveIK(const std::size_t record_idx)
{
  ReachRecord lookup = result_.at(record_idx);

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
  result_[record_idx] = lookup;
}

void ReachVisualizer::showResult(const std::size_t record_idx) const
{
  ReachRecord lookup = result_.at(record_idx);
  display_->updateRobotPose(lookup.goal_state);
}

void ReachVisualizer::showSeed(const std::size_t record_idx) const
{
  ReachRecord lookup = result_.at(record_idx);
  display_->updateRobotPose(lookup.seed_state);
}

void ReachVisualizer::reachNeighbors(const std::size_t record_id, const bool recursive) const
{
  ReachRecord lookup = result_.at(record_id);
  std::map<std::size_t, ReachRecord> result;
  if (recursive)
  {
    NeighborReachResult neighbors;
    reachNeighborsRecursive(result_, lookup, solver_, evaluator_, neighbor_radius_, neighbors, search_tree_);
    std::transform(neighbors.reached_pts.begin(), neighbors.reached_pts.end(), std::inserter(result, result.begin()),
                   [this](const std::size_t idx) { return std::make_pair(idx, result_.at(idx)); });
  }
  else
  {
    result = reachNeighborsDirect(result_, lookup, solver_, evaluator_, neighbor_radius_, search_tree_);
  }

  display_->updateRobotPose(lookup.goal_state);
  display_->showReachNeighborhood(result);
}

}  // namespace reach
