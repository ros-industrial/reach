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
#include "tf2_eigen/tf2_eigen.h"

#include <reach_core/ik_helper.h>
#include <reach_core/utils/general_utils.h>

namespace reach {
namespace core {

std::vector<reach_msgs::msg::ReachRecord> getNeighbors(
    const reach_msgs::msg::ReachRecord &rec, const ReachDatabasePtr db,
    const double radius) {
  const float x = rec.goal.position.x;
  const float y = rec.goal.position.y;
  const float z = rec.goal.position.z;

  // Create vectors for storing poses and reach record messages that lie within
  // radius of current point
  std::vector<reach_msgs::msg::ReachRecord> reach_records;

  // Iterate through all points in database to find those that lie within radius
  // of current point
  for (auto it = db->begin(); it != db->end(); ++it) {
    float xp = it->second.goal.position.x;
    float yp = it->second.goal.position.y;
    float zp = it->second.goal.position.z;
    float d2 = std::pow((xp - x), 2.0f) + std::pow((yp - y), 2.0f) +
               std::pow((zp - z), 2.0f);

    if (d2 < std::pow(radius, 2.0f) && d2 != 0.0f) {
      reach_records.push_back(it->second);
    }
  }

  return reach_records;
}

std::vector<reach_msgs::msg::ReachRecord> getNeighborsFLANN(
    const reach_msgs::msg::ReachRecord &rec, const ReachDatabasePtr db,
    const double radius, SearchTreePtr search_tree) {
  flann::Matrix<double> query(new double[3], 1, 3);
  query[0][0] = (double)rec.goal.position.x;
  query[0][1] = (double)rec.goal.position.y;
  query[0][2] = (double)rec.goal.position.z;

  std::vector<std::vector<int>> indices;
  std::vector<std::vector<double>> distances;
  search_tree->radiusSearch(query, indices, distances, radius,
                            flann::SearchParams());

  // Create vectors for storing poses and reach record messages that lie within
  // radius of current point
  std::vector<reach_msgs::msg::ReachRecord> neighbors;

  for (std::size_t i = 0; i < indices.size(); ++i) {
    for (std::size_t j = 0; j < indices[i].size(); ++j) {
      auto it = db->begin();
      std::advance(it, j);
      neighbors.push_back(it->second);
    }
  }

  return neighbors;
}

NeighborReachResult reachNeighborsDirect(
    ReachDatabasePtr db, const reach_msgs::msg::ReachRecord &rec,
    reach::plugins::IKSolverBasePtr solver, const double radius,
    SearchTreePtr search_tree) {
  // Initialize return array of string IDs of msgs that have been updated
  NeighborReachResult result;

  // Get all of the neighboring points
  std::vector<reach_msgs::msg::ReachRecord> neighbors;
  if (search_tree) {
    neighbors = getNeighborsFLANN(rec, db, radius, search_tree);
  } else {
    neighbors = getNeighbors(rec, db, radius);
  }

  // Solve IK for points that lie within sphere
  if (!neighbors.empty()) {
    std::map<std::string, double> previous_solution;
    for (std::size_t i = 0; i < rec.goal_state.position.size(); ++i) {
      previous_solution.emplace(rec.goal_state.name[i],
                                rec.goal_state.position[i]);
    }

    const std::vector<double> &current_pose = rec.goal_state.position;

    for (std::size_t i = 0; i < neighbors.size(); ++i) {
      // Initialize new target pose and new empty robot goal state
      Eigen::Isometry3d target;
      tf2::fromMsg(neighbors[i].goal, target);

      // Use current point's IK solution as seed
      std::vector<double> new_solution;
      std::vector<double> new_cartesian_space_waypoints;
      std::vector<double> new_joint_space_trajectory;
      double new_fraction = 0.0;
      std::optional<double> score = solver->solveIKFromSeed(
          target, previous_solution, new_solution, new_joint_space_trajectory,
          new_cartesian_space_waypoints, new_fraction);

      if (score) {
        // Calculate the joint distance between the seed and new goal states
        for (std::size_t j = 0; j < current_pose.size(); ++j) {
          result.joint_distance += std::abs(new_solution[j] - current_pose[j]);
        }

        // Change database if currently solved point didn't have solution before
        // or if its current manipulability is better than that saved in the
        // databas
        reach_msgs::msg::ReachRecord msg = neighbors[i];

        if (!msg.reached || (*score > msg.score)) {
          // Overwrite Reach Record msg parameters with new results
          msg.reached = true;
          msg.seed_state.position = rec.goal_state.position;
          msg.goal_state.position = new_solution;
          msg.score = *score;
          msg.joint_space_trajectory = new_joint_space_trajectory;
          msg.waypoints = new_cartesian_space_waypoints;
          msg.retrieved_fraction = new_fraction;
          db->put(msg);
        }

        // Populate return array of changed points
        result.reached_pts.push_back(msg.id);
      }
    }
  }
  return result;
}

void reachNeighborsRecursive(ReachDatabasePtr db,
                             const reach_msgs::msg::ReachRecord &rec,
                             reach::plugins::IKSolverBasePtr solver,
                             const double radius, NeighborReachResult result,
                             SearchTreePtr search_tree) {
  // Add the current point to the output list of msg IDs
  result.reached_pts.push_back(rec.id);

  // Create vectors for storing reach record messages that lie within radius of
  // current point
  std::vector<reach_msgs::msg::ReachRecord> neighbors;
  if (search_tree) {
    neighbors = getNeighborsFLANN(rec, db, radius, search_tree);
  } else {
    neighbors = getNeighbors(rec, db, radius);
  }

  // Solve IK for points that lie within sphere
  if (neighbors.size() > 0) {
    for (std::size_t i = 0; i < neighbors.size(); ++i) {
      const std::vector<double> &current_pose = rec.goal_state.position;

      std::map<std::string, double> current_pose_map;
      for (std::size_t i = 0; i < current_pose.size(); ++i) {
        current_pose_map.emplace(rec.goal_state.name[i], current_pose[i]);
      }

      // Check if the current potential point has been solved previously in the
      // recursion chain
      if (std::find(result.reached_pts.begin(), result.reached_pts.end(),
                    neighbors[i].id) == std::end(result.reached_pts)) {
        std::vector<double> new_pose;
        Eigen::Isometry3d target;
        tf2::fromMsg(neighbors[i].goal, target);

        std::vector<double> cartesian_space_waypoints;
        std::vector<double> joint_space_trajectory;
        double fraction;
        // Use current point's IK solution as seed
        std::optional<double> score = solver->solveIKFromSeed(
            target, current_pose_map, new_pose, joint_space_trajectory,
            cartesian_space_waypoints, fraction);
        if (score) {
          // Calculate the joint distance between the seed and new goal states
          for (std::size_t j = 0; j < current_pose.size(); ++j) {
            result.joint_distance += std::abs(new_pose[j] - current_pose[j]);
          }

          // Store information in new reach record object
          reach_msgs::msg::ReachRecord new_rec(neighbors[i]);
          new_rec.seed_state.position = current_pose;
          new_rec.goal_state.position = new_pose;
          new_rec.reached = true;
          new_rec.score = *score;

          // Recursively enter this function at the new neighboring location
          reachNeighborsRecursive(db, new_rec, solver, radius, result,
                                  search_tree);
        }
      }
    }
  }
}

}  // namespace core
}  // namespace reach
