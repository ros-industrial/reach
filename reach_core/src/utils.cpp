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
std::tuple<std::vector<double>, double> evaluateIK(const Eigen::Isometry3d& target,
                                                   const std::map<std::string, double>& seed,
                                                   IKSolver::ConstPtr ik_solver, Evaluator::ConstPtr evaluator)
{
  std::vector<std::vector<double>> poses = ik_solver->solveIK(target, seed);
  const std::vector<std::string> joint_names = ik_solver->getJointNames();
  double best_score = 0.0;
  std::size_t best_idx = 0;

  for (std::size_t i = 0; i < poses.size(); ++i)
  {
    double score = evaluator->calculateScore(zip(joint_names, poses[i]));
    if (score > best_score)
    {
      best_score = score;
      best_idx = i;
    }
  }

  return std::make_tuple(poses.at(best_idx), best_score);
}

SearchTreePtr createSearchTree(const ReachDatabase& db)
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (auto it = db.begin(); it != db.end(); ++it)
  {
    pcl::PointXYZ pt;
    pt.getVector3fMap() = it->second.goal.translation().cast<float>();
    cloud->push_back(pt);
  }
  auto search_tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  search_tree->setInputCloud(cloud);

  return search_tree;
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

std::vector<ReachRecord> reachNeighborsDirect(ReachDatabase::ConstPtr db, const ReachRecord& rec,
                                              IKSolver::ConstPtr solver, Evaluator::ConstPtr evaluator,
                                              const double radius, SearchTreePtr search_tree)
{
  // Get all of the neighboring points
  std::vector<std::string> neighbor_ids;
  if (search_tree)
  {
    neighbor_ids = getNeighborsFLANN(rec, db, radius, search_tree);
  }
  else
  {
    neighbor_ids = getNeighbors(rec, db, radius);
  }
  if (neighbor_ids.empty())
    return {};

  std::vector<ReachRecord> neighbors;
  std::transform(neighbor_ids.begin(), neighbor_ids.end(), std::back_inserter(neighbors),
                 [&db](const std::string& key) { return db->get(key); });

  // Solve IK for points that lie within sphere
  auto it = neighbors.begin();
  while (it != neighbors.end())
  {
    try
    {
      // Use current point's IK solution as seed
      std::vector<double> new_solution;
      double score;
      std::tie(new_solution, score) = evaluateIK(it->goal, rec.goal_state, solver, evaluator);

      // Update the record
      if (!it->reached || score > it->score)
      {
        it->reached = true;
        it->seed_state = rec.goal_state;
        it->goal_state = zip(solver->getJointNames(), new_solution);
        it->score = score;
      }
      ++it;
    }
    catch (const std::exception&)
    {
      it = neighbors.erase(it);
      continue;
    }
  }

  return neighbors;
}

void reachNeighborsRecursive(ReachDatabase::ConstPtr db, const ReachRecord& rec, IKSolver::ConstPtr solver,
                             Evaluator::ConstPtr evaluator, const double radius, NeighborReachResult& result,
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
      if (std::find(result.reached_pts.begin(), result.reached_pts.end(), neighbors[i]) != result.reached_pts.end())
        continue;

      try
      {
        ReachRecord neighbor = db->get(neighbors[i]);

        // Use current point's IK solution as seed
        std::vector<double> new_pose;
        double score;
        std::tie(new_pose, score) = evaluateIK(neighbor.goal, rec.goal_state, solver, evaluator);

        // Store information in new reach record object
        //        neighbor.seed_state = rec.goal_state;
        //        neighbor.goal_state = zip(solver->getJointNames(), new_pose);
        //        neighbor.reached = true;
        //        neighbor.score = score;

        // Calculate the joint distance between the seed and new goal states
        for(const auto& pair : rec.goal_state)
        {
          result.joint_distance += std::abs(neighbor.goal_state.at(pair.first) - pair.second);
        }

        // Recursively enter this function at the new neighboring location
        reachNeighborsRecursive(db, neighbor, solver, evaluator, radius, result, search_tree);
      }
      catch (const std::exception&)
      {
        continue;
      }
    }
  }
}

}  // namespace reach
