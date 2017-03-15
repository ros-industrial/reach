#ifndef REACH_DATABASE_H
#define REACH_DATABASE_H

#include <robot_reach_study/ReachDatabase.h>
#include <boost/optional.hpp>
#include <unordered_map>
#include <moveit/robot_state/robot_state.h>
#include <mutex>

namespace robot_reach_study
{

robot_reach_study::ReachRecord
makeRecordSuccess(const std::string& id, const geometry_msgs::Pose& goal,
                  const moveit::core::RobotState& seed_state,
                  const moveit::core::RobotState& goal_state,
                  const double score);

robot_reach_study::ReachRecord
makeRecordFailure(const std::string& id, const geometry_msgs::Pose& goal,
                  const moveit::core::RobotState& seed_state,
                  const double score);


class Database
{
public:
  using iterator = std::unordered_map<std::string, robot_reach_study::ReachRecord>::iterator;
  Database() = default;

  // Database disk i/o
  void save(const std::string& filename) const;

  bool load(const std::string& filename);

  // Database API
  boost::optional<robot_reach_study::ReachRecord> get(const std::string& id) const;

  void put(const robot_reach_study::ReachRecord& record);

  int count();

  void calculateResults();

  void printResults();

  float getTotalScore() const {return total_score_;}

  float getNormalizedTotalScore() const {return norm_total_score_;}

  float getAverageNeighborsCount() const {return avg_neighbors_;}

  void setAverageNeighborsCount(const float n) {avg_neighbors_ = n;}

  float getReachPercentage() const {return reach_percentage_;}

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
  float total_score_ = 0.0f;
  float norm_total_score_ = 0.0f;
  float avg_neighbors_ = 0.0f;
  float reach_percentage_ = 0.0f;
};

}

#endif // REACH_DATABASE_H
