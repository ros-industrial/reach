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

//#include <boost/multi_index_container.hpp>
//#include <boost/multi_index/sequenced_index.hpp>
//#include <boost/multi_index/ordered_index.hpp>
//#include <boost/multi_index/indexed_by.hpp>
//#include <boost/multi_index/member.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <Eigen/Dense>
#include <mutex>

namespace boost
{
namespace serialization
{
template <class Archive>
inline void serialize(Archive& ar, Eigen::Isometry3d& pose, const unsigned int version)
{
  std::vector<double> position(3);
  Eigen::Map<Eigen::Vector3d> position_map(position.data());
  position_map = pose.translation();

  std::vector<double> quaternion(4);
  Eigen::Map<Eigen::Quaterniond> q_map(quaternion.data());
  q_map = Eigen::Quaterniond(pose.linear());

  ar& BOOST_SERIALIZATION_NVP(position);
  ar& BOOST_SERIALIZATION_NVP(quaternion);
}

} // namespace serialization
} // namespace boost

namespace reach
{
//namespace bmi = boost::multi_index;

//typedef std::pair<std::string, double> Joint;
//typedef bmi::multi_index_container<
//    Joint, bmi::indexed_by<bmi::sequenced<>, bmi::ordered_unique<bmi::member<Joint, std::string, &Joint::first>>>>
//    JointState;
///** @brief Typedef for accessing the joint container as a sequenced list */
//typedef bmi::nth_index<JointState, 0>::type JointStateList;
///** @brief Typedef for accessing the joint container as a map-style structure */
//typedef bmi::nth_index<JointState, 1>::type JointStateMap;

class ReachRecord
{
public:
  ReachRecord() = default;
  ReachRecord(const std::string id, const bool reached, const Eigen::Isometry3d& goal,
              const std::map<std::string, double> seed_state, const std::map<std::string, double> goal_state,
              const double score);

  std::string id;
  bool reached;
  Eigen::Isometry3d goal;
  std::map<std::string, double> seed_state;
  std::map<std::string, double> goal_state;
  double score;

private:
  friend class boost::serialization::access;

  template <class Archive>
  inline void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& BOOST_SERIALIZATION_NVP(id);
    ar& BOOST_SERIALIZATION_NVP(reached);
    ar& BOOST_SERIALIZATION_NVP(goal);
    ar& BOOST_SERIALIZATION_NVP(seed_state);
    ar& BOOST_SERIALIZATION_NVP(goal_state);
    ar& BOOST_SERIALIZATION_NVP(score);
  }
};

/**
 * @brief The StudyResults struct
 */
class StudyResults
{
public:
  float total_pose_score = 0.0f;
  float norm_total_pose_score = 0.0f;
  float reach_percentage = 0.0f;

  inline std::string print()
  {
    std::stringstream ss;
    ss << "------------------------------------------------\n";
    ss << "Percent Reached = " << reach_percentage << "\n";
    ss << "Total points score = " << total_pose_score << "\n";
    ss << "Normalized total points score = " << norm_total_pose_score << "\n";
    ss << "------------------------------------------------\n";
    return ss.str();
  }
};

/**
 * @brief The Database class stores information about the robot pose for all of the attempted target poses. The database
 * also saves several key meta-results of the reach study:
 *  - reach_percentage: the percentage of all attempted poses that were reachable
 *  - avg_score: the average pose score for all reachable points (only significant relative to the score of a different
 * reach study)
 *  - norm_avg_score: average pose score divided by the reach percentage
 *  - avg_neighbors: average number of reachable neighbors from any given reachable pose (correlated with the size of
 * the robot's work area from a given pose, assuming the poses on the reach object are evenly distributed)
 *  - avg_joint_distance: average joint distance required to travel to all of any given pose's reachable neighbors
 * (indicative of the robot's ease of movement or "efficiency" moving from one pose to a neighboring pose
 */
class ReachDatabase
{
  using iterator = std::map<std::string, ReachRecord>::iterator;
  using const_iterator = std::map<std::string, ReachRecord>::const_iterator;

public:
  using Ptr = std::shared_ptr<ReachDatabase>;
  using ConstPtr = std::shared_ptr<const ReachDatabase>;

  ReachDatabase(const std::string name = "reach_study");
  ReachDatabase(const ReachDatabase&);
  ReachDatabase& operator=(const ReachDatabase&);

  std::string getName() const;

  /**
   * @brief get returns a ReachRecord message from the database
   * @param id
   * @return
   */
  ReachRecord get(const std::string& id) const;

  /**
   * @brief put adds a ReachRecord message to the database
   * @param record
   */
  void put(const ReachRecord& record);

  /**
   * @brief count counts the number of entries in the database
   * @return
   */
  std::size_t size() const;

  /**
   * @brief calculateResults calculates the results of the reach study and saves them to internal class members
   */
  StudyResults calculateResults();

  Eigen::MatrixX3f computeHeatMapColors() const;

  // For loops
  iterator begin();
  const_iterator begin() const;
  iterator end();
  const_iterator end() const;

  // Max element iterators
  iterator max();
  const_iterator max() const;

private:
  std::string name_;
  std::map<std::string, ReachRecord> map_;

  mutable std::mutex mutex_;

  friend class boost::serialization::access;
  template <class Archive>
  inline void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& BOOST_SERIALIZATION_NVP(name_);
    ar& BOOST_SERIALIZATION_NVP(map_);
  }
};

void save(const ReachDatabase& db, const std::string& filename);

ReachDatabase load(const std::string& filename);

}  // namespace reach

#endif  // REACH_CORE_REACH_DATABASE_H
