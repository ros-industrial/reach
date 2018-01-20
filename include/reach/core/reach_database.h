#ifndef REACH_CORE_REACH_DATABASE_H
#define REACH_CORE_REACH_DATABASE_H

#include <reach/core/study_parameters.h>
#include <robot_reach_study/ReachDatabase.h>
#include <boost/optional.hpp>
#include <moveit/robot_state/robot_state.h>
#include <mutex>
#include <unordered_map>

namespace reach
{
namespace core
{

/**
 * @brief The Database class stores information about the robot pose for all of the attempted target poses. The database also saves
 * several key meta-results of the reach study:
 *  - reach_percentage: the percentage of all attempted poses that were reachable
 *  - avg_score: the average pose score for all reachable points (only significant relative to the score of a different reach study)
 *  - norm_avg_score: average pose score divided by the reach percentage
 *  - avg_neighbors: average number of reachable neighbors from any given reachable pose (correlated with the size of the robot's work
 *    area from a given pose, assuming the poses on the reach object are evenly distributed)
 *  - avg_joint_distance: average joint distance required to travel to all of any given pose's reachable neighbors (indicative of the
 *    robot's ease of movement or "efficiency" moving from one pose to a neighboring pose
 */
class ReachDatabase
{
  using iterator = std::unordered_map<std::string, robot_reach_study::ReachRecord>::iterator;

public:

  /**
    @brief Default class constructor
   */
  ReachDatabase() = default;

  /**
   * @brief save saves the reach study database to a file at the input location
   * @param filename
   */
  void save(const std::string& filename) const;

  /**
   * @brief load loads a saved reach study database from the input location
   * @param filename
   * @return true on success, false on failure
   */
  bool load(const std::string& filename);

  /**
   * @brief get returns a ReachRecord message from the database
   * @param id
   * @return
   */
  boost::optional<robot_reach_study::ReachRecord> get(const std::string& id) const;

  /**
   * @brief put adds a ReachRecord message to the database
   * @param record
   */
  void put(const robot_reach_study::ReachRecord& record);

  /**
   * @brief count counts the number of entries in the database
   * @return
   */
  int count();

  /**
   * @brief calculateResults calculates the results of the reach study and saves them to internal class members
   */
  void calculateResults();

  /**
   * @brief printResults prints the calculated results of the reach study to the terminal
   */
  void printResults();

//  /**
//   * @brief getTotalPoseScore
//   * @return
//   */
//  float getTotalPoseScore() const {return total_pose_score_;}

//  /**
//   * @brief getNormalizedTotalPoseScore
//   * @return
//   */
//  float getNormalizedTotalPoseScore() const {return norm_total_pose_score_;}

//  /**
//   * @brief getAverageNeighborsCount
//   * @return
//   */
//  float getAverageNeighborsCount() const {return avg_num_neighbors_;}

//  /**
//   * @brief getReachPercentage
//   * @return
//   */
//  float getReachPercentage() const {return reach_percentage_;}

//  /**
//   * @brief getAverageJointDistance
//   * @return
//   */
//  float getAverageJointDistance() const {return avg_joint_distance_;}

  /**
   * @brief getStudyResults
   * @return
   */
  StudyResults getStudyResults() const
  {
    return results_;
  }

  /**
   * @brief setAverageNeighborsCount
   * @param n
   */
  void setAverageNeighborsCount(const float n) {results_.avg_num_neighbors = n;}

  /**
   * @brief setAverageJointDistance
   * @param n
   */
  void setAverageJointDistance(const float n) {results_.avg_joint_distance = n;}

  // For loops
  iterator begin()
  {
    return map_.begin();
  }

  iterator end()
  {
    return map_.end();
  }

private:

  void putHelper(const robot_reach_study::ReachRecord& record);

  std::unordered_map<std::string, robot_reach_study::ReachRecord> map_;

  mutable std::mutex mutex_;

  StudyResults results_;

};

} // namespace core
} // namespace reach

#endif // REACH_CORE_REACH_DATABASE_H
