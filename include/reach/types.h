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
#ifndef reach_TYPES_H
#define reach_TYPES_H

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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ReachRecord(const bool reached, const Eigen::Isometry3d& goal, const std::map<std::string, double> seed_state,
              const std::map<std::string, double> goal_state, const double score);
  ReachRecord() = default;
  ReachRecord(const ReachRecord&) = default;
  ReachRecord& operator=(const ReachRecord&) = default;
  ReachRecord& operator=(ReachRecord&&) = default;

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
class ReachResultSummary
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
using ReachResult = std::vector<ReachRecord, Eigen::aligned_allocator<ReachRecord>>;
using VectorReachResult = std::vector<ReachResult, Eigen::aligned_allocator<ReachResult>>;

ReachResultSummary calculateResults(const ReachResult& db);

/**
 * @brief Returns a vector of normalized reach target scores
 * @details If use_full_range is false (default), the individual scores are divided by the maximum
 * score. If use_full_range is true, the individual scores are fully normalized on [0, 1]
 */
std::vector<float> normalizeScores(const ReachResult& result, bool use_full_range);

/**
 * @brief Computes the colors for a heat map based on the input scores
 * @param scores Vector of reach target scores in the range [0, 1]
 * @return An array of RGB colors on [0, 1] for each channel
 */
Eigen::MatrixX3f computeHeatMapColors(const std::vector<float>& scores, float hue_low_score = 270.0f,
                                      float hue_high_score = 0.0f);

/**
 * @brief Computes heat map colors for the reach targets in a reach study result
 * @details The heat map colors range from deep blue for the lowest score (i.e., coldest) to deep red for the highest
 * score (i.e., hottest). If use_full_color_range is false (default), the individual scores are divided by the maximum
 * score before colorization. If use_full_color_range is true, the individual scores are fully normalized on [0, 1]
 * before colorization
 * @return An array of RGB colors on [0, 1] for each channel
 */
Eigen::MatrixX3f computeHeatMapColors(const ReachResult& result, bool use_full_color_range,
                                      float hue_low_score = 270.0f, float hue_high_score = 0.0f);

class ReachDatabase
{
public:
  /** @brief Results by reach study iteration */
  VectorReachResult results;

  bool operator==(const ReachDatabase& rhs) const;
  ReachResultSummary calculateResults() const;
  Eigen::MatrixX3f computeHeatMapColors(bool use_full_color_range = false, float hue_low_score = 270.0f,
                                        float hue_high_score = 0.0f) const;

private:
  friend class boost::serialization::access;

  template <class Archive>
  inline void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& BOOST_SERIALIZATION_NVP(results);
  }
};

void save(const ReachDatabase& db, const std::string& filename);

ReachDatabase load(const std::string& filename);

}  // namespace reach

#endif  // reach_TYPES_H
