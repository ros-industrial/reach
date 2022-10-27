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

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
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

namespace reach
{
ReachRecord::ReachRecord(const std::string id_, const bool reached_, const Eigen::Isometry3d& goal_,
                         const std::map<std::string, double> seed_state_,
                         const std::map<std::string, double> goal_state_, const double score_)
  : id(id_), reached(reached_), goal(goal_), seed_state(seed_state_), goal_state(goal_state_), score(score_)
{
}

ReachDatabase::ReachDatabase(const std::string name) : name_(std::move(name))
{
}

ReachDatabase::ReachDatabase(const ReachDatabase& rhs) : name_(rhs.name_), map_(rhs.map_)
{
}

ReachDatabase& ReachDatabase::operator=(const ReachDatabase& rhs)
{
  name_ = rhs.name_;
  map_ = rhs.map_;
  return *this;
}

std::string ReachDatabase::getName() const
{
  return name_;
}

ReachRecord ReachDatabase::get(const std::string& id) const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  return map_.at(id);
}

void ReachDatabase::put(const ReachRecord& record)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  map_[record.id] = record;
}

std::size_t ReachDatabase::size() const
{
  return map_.size();
}

ReachDatabase::iterator ReachDatabase::begin()
{
  return map_.begin();
}

ReachDatabase::const_iterator ReachDatabase::begin() const
{
  return map_.cbegin();
}

ReachDatabase::iterator ReachDatabase::end()
{
  return map_.end();
}

ReachDatabase::const_iterator ReachDatabase::end() const
{
  return map_.cend();
}

ReachDatabase::iterator ReachDatabase::max()
{
  using RecordPair = std::map<std::string, ReachRecord>::value_type;
  return std::max_element(map_.begin(), map_.end(),
                          [](const RecordPair& a, const RecordPair& b) { return a.second.score < b.second.score; });
}

ReachDatabase::const_iterator ReachDatabase::max() const
{
  return max();
}

StudyResults ReachDatabase::calculateResults() const
{
  unsigned int success = 0, total = 0;
  double score = 0.0;
  for (std::size_t i = 0; i < this->size(); ++i)
  {
    ReachRecord msg = get(std::to_string(i));

    if (msg.reached)
    {
      success++;
      score += msg.score;
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
  boost::archive::binary_oarchive oa(ofs);
  oa << db;
}

ReachDatabase load(const std::string& filename)
{
  std::ifstream ifs(filename);
  boost::archive::binary_iarchive ia(ifs);
  ReachDatabase db;
  ia >> db;
  return db;
}

Eigen::MatrixX3f ReachDatabase::computeHeatMapColors() const
{
  // Find the max element
  ReachDatabase::const_iterator max_it = max();

  Eigen::MatrixX3f colors(map_.size(), 3);
  for (auto it = map_.begin(); it != map_.end(); ++it)
  {
    // Compute the color of the marker as a heatmap from blue to red using HSV space
    const float max_h = 0.75f * 360.0f;  // Corresponds to blue color
    const float h = max_h - (static_cast<float>(it->second.score / max_it->second.score) * max_h);
    const float s = 1.0f;
    const float v = it->second.reached ? 1.0f : 0.0f;

    // Convert to RGB
    const pcl::PointXYZHSV pt_hsv(h, s, v);
    pcl::PointXYZRGB pt_rgb;
    pcl::PointXYZHSVtoXYZRGB(pt_hsv, pt_rgb);

    Eigen::Index idx = static_cast<Eigen::Index>(std::distance(map_.begin(), it));
    colors.row(idx) = pt_rgb.getRGBVector3i().cast<float>() / 255.0f;
  }

  return colors;
}

}  // namespace reach
