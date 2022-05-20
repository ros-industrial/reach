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
#ifndef REACH_CORE_REACH_DATABASE_H
#define REACH_CORE_REACH_DATABASE_H

#include "reach_core/study_parameters.h"

#include <map>
#include <mutex>
#include <optional>
#include <unordered_map>

#include <reach_msgs/msg/reach_database.hpp>

namespace reach {
namespace core {

/**
 * @brief makeRecord
 * @param id
 * @param reached
 * @param goal
 * @param seed_state
 * @param goal_state
 * @param score
 * @return
 */
reach_msgs::msg::ReachRecord makeRecord(
    const std::string &id, const bool reached,
    const geometry_msgs::msg::Pose &goal,
    const sensor_msgs::msg::JointState &seed_state,
    const sensor_msgs::msg::JointState &goal_state, const double score,
    const std::string &ik_solver_name, const std::vector<double> &waypoints,
    const std::vector<double> &trajectory, double retrieved_fraction);

/**
 * @brief toMap
 * @param state
 * @return
 */
std::map<std::string, double> jointStateMsgToMap(
    const sensor_msgs::msg::JointState &state);

/**
 * @brief to vector of Map
 * @param trectory
 * @return
 */
std::vector<std::map<std::string, double>> jointStateArrayToArrayOfMaps(
    const std::vector<double> &trajectory,
    const std::vector<std::string> &names);
/**
 * @brief The Database class stores information about the robot pose for all of
 * the attempted target poses. The database also saves several key meta-results
 * of the reach study:
 *  - reach_percentage: the percentage of all attempted poses that were
 * reachable
 *  - avg_score: the average pose score for all reachable points (only
 * significant relative to the score of a different reach study)
 *  - norm_avg_score: average pose score divided by the reach percentage
 *  - avg_neighbors: average number of reachable neighbors from any given
 * reachable pose (correlated with the size of the robot's work area from a
 * given pose, assuming the poses on the reach object are evenly distributed)
 *  - avg_joint_distance: average joint distance required to travel to all of
 * any given pose's reachable neighbors (indicative of the robot's ease of
 * movement or "efficiency" moving from one pose to a neighboring pose
 */
class ReachDatabase {
  using iterator =
      std::unordered_map<std::string, reach_msgs::msg::ReachRecord>::iterator;

 public:
  /**
@brief Default class constructor
*/
  ReachDatabase() = default;

  /**
   * @brief save saves the reach study database to a file at the input location
   * @param filename
   */
  void save(const std::string &filename) const;

  /**
   * @brief load loads a saved reach study database from the input location
   * @param filename
   * @return true on success, false on failure
   */
  bool load(const std::string &filename);

  /**
   * @brief get returns a ReachRecord message from the database
   * @param id
   * @return
   */
  std::optional<reach_msgs::msg::ReachRecord> get(const std::string &id) const;

  /**
   * @brief put adds a ReachRecord message to the database
   * @param record
   */
  void put(const reach_msgs::msg::ReachRecord &record);

  /**
   * @brief count counts the number of entries in the database
   * @return
   */
  std::size_t size() const;

  /**
   * @brief calculateResults calculates the results of the reach study and saves
   * them to internal class members
   */
  void calculateResults();

  /**
   * @brief printResults prints the calculated results of the reach study to the
   * terminal
   */
  void printResults();

  /**
   * @brief getStudyResults
   * @return
   */
  StudyResults getStudyResults() const { return results_; }

  /**
   * @brief setAverageNeighborsCount
   * @param n
   */
  void setAverageNeighborsCount(const float n) {
    results_.avg_num_neighbors = n;
  }

  /**
   * @brief setAverageJointDistance
   * @param n
   */
  void setAverageJointDistance(const float n) {
    results_.avg_joint_distance = n;
  }

  // For loops
  iterator begin() { return map_.begin(); }

  iterator end() { return map_.end(); }

  reach_msgs::msg::ReachDatabase toReachDatabaseMsg();

 private:
  void putHelper(const reach_msgs::msg::ReachRecord &record);

  std::unordered_map<std::string, reach_msgs::msg::ReachRecord> map_;

  mutable std::mutex mutex_;

  StudyResults results_;
};
typedef std::shared_ptr<ReachDatabase> ReachDatabasePtr;

}  // namespace core
}  // namespace reach

#endif  // REACH_CORE_REACH_DATABASE_H
