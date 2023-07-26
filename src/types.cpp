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
#include <reach/types.h>

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

ReachResultSummary calculateResults(const ReachResult& db)
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

  ReachResultSummary results;
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

  if (db.results.empty())
    throw std::runtime_error("Database has no results");

  return db;
}

Eigen::MatrixX3f computeHeatMapColors(const std::vector<float>& scores, float hue_low_score, float hue_high_score)
{
  // Compute the color of the marker as a color scale in HSV space based on the given boundary hue values
  Eigen::MatrixX3f colors(scores.size(), 3);
  for (std::size_t i = 0; i < scores.size(); ++i)
  {
    // Convert to RGB (note: constructor seems to have strange behavior, so set HSV individually)
    pcl::PointXYZHSV pt_hsv;
    // The following computation of the hue also works if hue_high_score < hue_low_score
    pt_hsv._PointXYZHSV::h = hue_low_score + (hue_high_score - hue_low_score) * scores[i];
    pt_hsv._PointXYZHSV::s = 1.0f;
    pt_hsv._PointXYZHSV::v = scores[i] > std::numeric_limits<float>::epsilon() ? 1.0f : 0.0f;

    pcl::PointXYZRGB pt_rgb;
    pcl::PointXYZHSVtoXYZRGB(pt_hsv, pt_rgb);

    Eigen::Index idx = static_cast<Eigen::Index>(i);
    colors.row(idx) = pt_rgb.getRGBVector3i().cast<float>() / 255.0f;
  }

  return colors;
}

std::vector<float> normalizeScores(const ReachResult& result, bool use_full_range)
{
  // Find the max element
  auto max_it = std::max_element(result.begin(), result.end(),
                                 [](const ReachRecord& a, const ReachRecord& b) { return a.score < b.score; });

  // Find the min element
  float min_score = std::numeric_limits<float>::infinity();
  for (auto it = result.begin(); it != result.end(); ++it)
    if (it->reached && it->score < min_score)
      min_score = it->score;

  std::vector<float> scores;
  scores.reserve(result.size());

  std::transform(result.begin(), result.end(), std::back_inserter(scores), [&](const ReachRecord& r) {
    if (use_full_range)
      return static_cast<float>((r.score - min_score) / (max_it->score - min_score));
    else
      return static_cast<float>(r.score / max_it->score);
  });

  return scores;
}

Eigen::MatrixX3f computeHeatMapColors(const ReachResult& result, bool use_full_color_range, float hue_low_score,
                                      float hue_high_score)
{
  return computeHeatMapColors(normalizeScores(result, use_full_color_range), hue_low_score, hue_high_score);
}

bool ReachDatabase::operator==(const ReachDatabase& rhs) const
{
  return std::equal(results.begin(), results.end(), rhs.results.begin());
}

ReachResultSummary ReachDatabase::calculateResults() const
{
  if (results.empty())
    throw std::runtime_error("Database contains no results");

  return reach::calculateResults(results.back());
}

Eigen::MatrixX3f ReachDatabase::computeHeatMapColors(bool use_full_color_range, float hue_low_score,
                                                     float hue_high_score) const
{
  if (results.empty())
    throw std::runtime_error("Database contains no results");

  return reach::computeHeatMapColors(results.back(), use_full_color_range, hue_low_score, hue_high_score);
}

}  // namespace reach
