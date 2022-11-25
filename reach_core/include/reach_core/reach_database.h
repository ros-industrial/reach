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

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <Eigen/Dense>

namespace reach
{
using VectorIsometry3d = std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;

/** @brief Reachability data for a single target Cartesian pose */
class ReachRecord
{
public:
  ReachRecord() = default;
  ReachRecord(const bool reached, const Eigen::Isometry3d& goal,
              const std::map<std::string, double> seed_state, const std::map<std::string, double> goal_state,
              const double score);

  bool operator==(const ReachRecord& rhs) const;

  /** @brief Boolean flag indicating whether the target pose was reachable */
  bool reached;

  /** @brief Cartesian pose of the target */
  Eigen::Isometry3d goal;

  /** @brief Seed state used for the IK solver at this target pose */
  std::map<std::string, double> seed_state;

  /** @brief Robot pose (i.e., IK solution) used to reach this target pose */
  std::map<std::string, double> goal_state;

  /** @brief Reachability score associated with this target pose */
  double score;

private:
  friend class boost::serialization::access;

  template <class Archive>
  inline void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& BOOST_SERIALIZATION_NVP(reached);
    ar& BOOST_SERIALIZATION_NVP(goal);
    ar& BOOST_SERIALIZATION_NVP(seed_state);
    ar& BOOST_SERIALIZATION_NVP(goal_state);
    ar& BOOST_SERIALIZATION_NVP(score);
  }
};

/**
 * @brief Container for the results of a reach study
 */
class StudyResults
{
public:
  /**
   * @brief The total pose score for all reachable points
   * @details This score is generally only significant relative to the score of a different reach study using the same
   * geometry and target points
   */
  float total_pose_score = 0.0f;

  /**
   * @brief The total pose score for all reachable points divided by the percentage of reachable points
   * @details This score represents the possible score if all points could be made reachable (e.g., by modifying the
   * robot geometry, etc.)
   */
  float norm_total_pose_score = 0.0f;

  /** @brief The percentage of all attempted target poses that were reachable */
  float reach_percentage = 0.0f;

  inline std::string print() const
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
 * @brief Container to store information about the robot poses for all of the attempted target poses
 */
using ReachDatabase = std::vector<ReachRecord>;
StudyResults calculateResults(const ReachDatabase& db);
Eigen::MatrixX3f computeHeatMapColors(const ReachDatabase& db);

void save(const ReachDatabase& db, const std::string& filename);

ReachDatabase load(const std::string& filename);

}  // namespace reach

#endif  // REACH_CORE_REACH_DATABASE_H
