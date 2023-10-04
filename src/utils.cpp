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
#include <reach/utils.h>

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

SearchTreePtr createSearchTree(const VectorIsometry3d& poses)
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const Eigen::Isometry3d& pose : poses)
  {
    pcl::PointXYZ pt;
    pt.getVector3fMap() = pose.translation().cast<float>();
    cloud->push_back(pt);
  }
  auto search_tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  search_tree->setInputCloud(cloud);

  return search_tree;
}

std::vector<std::size_t> getNeighbors(const ReachRecord& rec, const ReachResult& db, const double radius)
{
  // Create vectors for storing poses and reach record messages that lie within radius of current point
  std::vector<std::size_t> neighbors;

  // Iterate through all points in database to find those that lie within radius of current point
  for (std::size_t i = 0; i < db.size(); ++i)
  {
    const ReachRecord& target = db[i];
    double d2 = (rec.goal.translation() - target.goal.translation()).squaredNorm();

    if (d2 < std::pow(radius, 2.0) && d2 > std::numeric_limits<double>::epsilon())
    {
      neighbors.push_back(i);
    }
  }

  return neighbors;
}

std::vector<std::size_t> getNeighborsFLANN(const ReachRecord& rec, const ReachResult& db, const double radius,
                                           SearchTreePtr search_tree)
{
  pcl::PointXYZ query(rec.goal.translation().x(), rec.goal.translation().y(), rec.goal.translation().z());
  std::vector<int> indices;
  std::vector<float> distances;
  search_tree->radiusSearch(query, radius, indices, distances);

  std::vector<std::size_t> neighbors;
  neighbors.reserve(indices.size());
  std::transform(indices.begin(), indices.end(), std::back_inserter(neighbors),
                 [](const int idx) { return static_cast<std::size_t>(idx); });

  return neighbors;
}

std::map<std::size_t, ReachRecord> reachNeighborsDirect(const ReachResult& db, const ReachRecord& rec,
                                                        IKSolver::ConstPtr solver, Evaluator::ConstPtr evaluator,
                                                        const double radius, SearchTreePtr search_tree)
{
  // Get all of the neighboring points
  std::vector<std::size_t> neighbor_idxs;
  if (search_tree)
  {
    neighbor_idxs = getNeighborsFLANN(rec, db, radius, search_tree);
  }
  else
  {
    neighbor_idxs = getNeighbors(rec, db, radius);
  }
  if (neighbor_idxs.empty())
    return {};

  std::map<std::size_t, ReachRecord> neighbors;
  std::transform(neighbor_idxs.begin(), neighbor_idxs.end(), std::inserter(neighbors, neighbors.begin()),
                 [&db](const std::size_t& idx) { return std::make_pair(idx, db.at(idx)); });

  // Solve IK for points that lie within sphere
  auto it = neighbors.begin();
  while (it != neighbors.end())
  {
    try
    {
      // Use current point's IK solution as seed
      std::vector<double> new_solution;
      double score;
      std::tie(new_solution, score) = evaluateIK(it->second.goal, rec.goal_state, solver, evaluator);

      // Update the record
      if (!it->second.reached || score > it->second.score)
      {
        it->second.reached = true;
        it->second.seed_state = rec.goal_state;
        it->second.goal_state = zip(solver->getJointNames(), new_solution);
        it->second.score = score;
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

void reachNeighborsRecursive(const ReachResult& db, const ReachRecord& rec, IKSolver::ConstPtr solver,
                             Evaluator::ConstPtr evaluator, const double radius, NeighborReachResult& result,
                             SearchTreePtr search_tree)
{
  // Add the current point to the output list of msg IDs
  //  result.reached_pts.push_back(rec.id);

  // Create vectors for storing reach record messages that lie within radius of current point
  std::vector<std::size_t> neighbors;
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
        ReachRecord neighbor = db.at(neighbors[i]);

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
        for (const auto& pair : rec.goal_state)
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

std::vector<double> extractSubset(const std::map<std::string, double>& input, const std::vector<std::string>& keys)
{
  if (keys.size() > input.size())
    throw std::runtime_error("Input map size was not at least as large as the number of keys");

  // Pull the joints of the planning group out of the input map
  std::vector<double> values;
  values.reserve(keys.size());
  for (const std::string& name : keys)
  {
    const auto it = input.find(name);
    if (it == input.end())
      throw std::runtime_error("Key '" + name + "' is not in the input map");

    values.push_back(it->second);
  }

  return values;
}

}  // namespace reach
