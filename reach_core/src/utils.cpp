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
#include <reach_core/utils.h>

#include <iostream>

namespace reach
{
void integerProgressPrinter(std::atomic<int>& current_counter, std::atomic<int>& previous_pct, const int total_size)
{
  const float current_pct_float = (static_cast<float>(current_counter.load()) / static_cast<float>(total_size)) * 100.0;
  const int current_pct = static_cast<int>(current_pct_float);
  if (current_pct > previous_pct.load())
  {
    std::cout << "[" << current_pct << "%]" << std::endl;
  }
  previous_pct = current_pct;
}

std::vector<std::string> getNeighbors(const ReachRecord& rec, const ReachDatabase::ConstPtr db, const double radius)
{
  // Create vectors for storing poses and reach record messages that lie within radius of current point
  std::vector<std::string> reach_records;

  // Iterate through all points in database to find those that lie within radius of current point
  for (auto it = db->begin(); it != db->end(); ++it)
  {
    double d2 = (rec.goal.translation() - it->second.goal.translation()).squaredNorm();

    if (d2 < std::pow(radius, 2.0) && d2 > std::numeric_limits<double>::epsilon())
    {
      reach_records.push_back(it->first);
    }
  }

  return reach_records;
}

std::vector<std::string> getNeighborsFLANN(const ReachRecord& rec, const ReachDatabase::ConstPtr db,
                                           const double radius, SearchTreePtr search_tree)
{
  pcl::PointXYZ query(rec.goal.translation().x(), rec.goal.translation().y(), rec.goal.translation().z());
  std::vector<int> indices;
  std::vector<float> distances;
  search_tree->radiusSearch(query, radius, indices, distances);

  std::vector<std::string> neighbors;
  for (std::size_t i = 0; i < indices.size(); ++i)
  {
    auto it = db->begin();
    std::advance(it, indices[i]);
    neighbors.push_back(it->first);
  }

  return neighbors;
}

NeighborReachResult reachNeighborsDirect(ReachDatabase::Ptr db, const ReachRecord& rec,
                                         IKSolver::ConstPtr solver, const double radius,
                                         SearchTreePtr search_tree)
{
  // Initialize return array of string IDs of msgs that have been updated
  NeighborReachResult result;

  // Get all of the neighboring points
  std::vector<std::string> neighbors;
  if (search_tree)
  {
    neighbors = getNeighborsFLANN(rec, db, radius, search_tree);
  }
  else
  {
    neighbors = getNeighbors(rec, db, radius);
  }

  // Solve IK for points that lie within sphere
  if (!neighbors.empty())
  {
    for (std::size_t i = 0; i < neighbors.size(); ++i)
    {
      try
      {
        ReachRecord neighbor = db->get(neighbors[i]);

        // Use current point's IK solution as seed
        std::vector<double> new_solution;
        double score;
        std::tie(new_solution, score) = solver->solveIKFromSeed(neighbor.goal, rec.goal_state);

        // Change database if currently solved point didn't have solution before
        // or if its current score is better than that saved in the databas
        if (!neighbor.reached || (score > neighbor.score))
        {
          // Overwrite Reach Record msg parameters with new results
          neighbor.reached = true;
          neighbor.seed_state = rec.goal_state;
          neighbor.goal_state = zip(solver->getJointNames(), new_solution);
          neighbor.score = score;
          db->put(neighbor);
        }

        // Populate return array of changed points
        result.reached_pts.push_back(neighbor.id);
      }
      catch (const std::exception&)
      {
        continue;
      }
    }
  }
  return result;
}

void reachNeighborsRecursive(ReachDatabase::Ptr db, const ReachRecord& rec,
                             IKSolver::ConstPtr solver, const double radius, NeighborReachResult result,
                             SearchTreePtr search_tree)
{
  // Add the current point to the output list of msg IDs
  result.reached_pts.push_back(rec.id);

  // Create vectors for storing reach record messages that lie within radius of current point
  std::vector<std::string> neighbors;
  if (search_tree)
  {
    neighbors = getNeighborsFLANN(rec, db, radius, search_tree);
  }
  else
  {
    neighbors = getNeighbors(rec, db, radius);
  }

  // Solve IK for points that lie within sphere
  if (neighbors.size() > 0)
  {
    for (std::size_t i = 0; i < neighbors.size(); ++i)
    {
      // Check if the current potential point has been solved previously in the recursion chain
      if (std::find(result.reached_pts.begin(), result.reached_pts.end(), neighbors[i]) != std::end(result.reached_pts))
        continue;

      try
      {
        ReachRecord neighbor = db->get(neighbors[i]);

        // Use current point's IK solution as seed
        std::vector<double> new_pose;
        double score;
        std::tie(new_pose, score) = solver->solveIKFromSeed(neighbor.goal, rec.goal_state);

        // Store information in new reach record object
        neighbor.seed_state = rec.goal_state;
        neighbor.goal_state = zip(solver->getJointNames(), new_pose);
        neighbor.reached = true;
        neighbor.score = score;

        // Calculate the joint distance between the seed and new goal states
        for(const auto& pair : rec.goal_state)
        {
          result.joint_distance += std::abs(neighbor.goal_state.at(pair.first) - pair.second);
        }

        // Recursively enter this function at the new neighboring location
        reachNeighborsRecursive(db, neighbor, solver, radius, result, search_tree);
      }
      catch (const std::exception&)
      {
        continue;
      }
    }
  }
}

}  // namespace reach
