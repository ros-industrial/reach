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
#include <reach_core/reach_database.h>

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <fstream>
#include <pcl/point_types_conversion.h>

namespace boost
{
namespace serialization
{
template <class Archive>
void save(Archive& ar, const Eigen::Isometry3d& pose, const unsigned int /*version*/)
{
  std::vector<double> matrix(pose.data(), pose.data() + 16);
  ar& BOOST_SERIALIZATION_NVP(matrix);
}

template <class Archive>
inline void load(Archive& ar, Eigen::Isometry3d& pose, const unsigned int /*version*/)
{
  std::vector<double> matrix(16);
  Eigen::Map<Eigen::Matrix4d> pose_map(matrix.data());
  ar& BOOST_SERIALIZATION_NVP(matrix);
  pose.matrix() = pose_map;
}

}  // namespace serialization
}  // namespace boost

BOOST_SERIALIZATION_SPLIT_FREE(Eigen::Isometry3d)

bool isApprox(const std::map<std::string, double>& lhs, const std::map<std::string, double>& rhs)
{
  if (lhs.size() != rhs.size())
    return false;

  for (auto lhs_it = lhs.begin(); lhs_it != lhs.end(); ++lhs_it)
  {
    auto rhs_it = rhs.begin();
    std::advance(rhs_it, std::distance(lhs.begin(), lhs_it));

    // Ensure the keys match
    if (lhs_it->first != rhs_it->first)
      return false;

    // Ensure the values are approximately equal
    if (std::abs(lhs_it->second - rhs_it->second) > std::numeric_limits<double>::epsilon())
      return false;
  }

  return true;
}

namespace reach
{
ReachRecord::ReachRecord(const bool reached_, const Eigen::Isometry3d& goal_,
                         const std::map<std::string, double> seed_state_,
                         const std::map<std::string, double> goal_state_, const double score_)
  : reached(reached_), goal(goal_), seed_state(seed_state_), goal_state(goal_state_), score(score_)
{
}

bool ReachRecord::operator==(const ReachRecord& rhs) const
{
  const bool reach_match = reached == rhs.reached;
  const bool goals_match = goal.isApprox(rhs.goal);
  const bool goal_states_match = isApprox(goal_state, rhs.goal_state);
  const bool seed_states_match = isApprox(seed_state, rhs.seed_state);
  const bool scores_match = std::abs(score - rhs.score) < std::numeric_limits<double>::epsilon();

  return reach_match && goals_match && goal_states_match && seed_states_match && scores_match;
}

StudyResults calculateResults(const ReachDatabase& db)
{
  unsigned int success = 0, total = 0;
  double score = 0.0;
  for (const ReachRecord& rec : db)
  {
    if (rec.reached)
    {
      success++;
      score += rec.score;
    }

    total++;
  }
  const float pct_success = static_cast<float>(success) / static_cast<float>(total);

  StudyResults results;
  results.reach_percentage = 100.0f * pct_success;
  results.total_pose_score = score;
  results.norm_total_pose_score = score / pct_success;

  return results;
}

void save(const ReachDatabase& db, const std::string& filename)
{
  std::ofstream ofs(filename);
  boost::archive::xml_oarchive oa(ofs);
  oa << BOOST_SERIALIZATION_NVP(db);
}

ReachDatabase load(const std::string& filename)
{
  std::ifstream ifs(filename);
  boost::archive::xml_iarchive ia(ifs);
  ReachDatabase db;
  ia >> BOOST_SERIALIZATION_NVP(db);
  return db;
}

Eigen::MatrixX3f computeHeatMapColors(const ReachDatabase& db)
{
  // Find the max element
  ReachDatabase::const_iterator max_it = std::max_element(
      db.begin(), db.end(), [](const ReachRecord& a, const ReachRecord& b) {
        return a.score < b.score;
      });

  Eigen::MatrixX3f colors(db.size(), 3);
  for (auto it = db.begin(); it != db.end(); ++it)
  {
    // Compute the color of the marker as a heatmap from blue to red using HSV space
    const float max_h = 0.75f * 360.0f;  // Corresponds to blue color
    const float h = max_h - (static_cast<float>(it->score / max_it->score) * max_h);
    const float s = 1.0f;
    const float v = it->reached ? 1.0f : 0.0f;

    // Convert to RGB
    const pcl::PointXYZHSV pt_hsv(h, s, v);
    pcl::PointXYZRGB pt_rgb;
    pcl::PointXYZHSVtoXYZRGB(pt_hsv, pt_rgb);

    Eigen::Index idx = static_cast<Eigen::Index>(std::distance(db.begin(), it));
    colors.row(idx) = pt_rgb.getRGBVector3i().cast<float>() / 255.0f;
  }

  return colors;
}

}  // namespace reach
