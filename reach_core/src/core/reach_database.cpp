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

namespace reach
{
namespace core
{
ReachRecord::ReachRecord(const std::string id_, const bool reached_, const Eigen::Isometry3d& goal_,
                         const std::map<std::string, double> seed_state_,
                         const std::map<std::string, double> goal_state_, const double score_)
  : id(id_), goal(goal_), reached(reached_), seed_state(seed_state_), goal_state(goal_state_), score(score_)
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

StudyResults ReachDatabase::calculateResults()
{
  unsigned int success = 0, total = 0;
  double score = 0.0;
  for (int i = 0; i < this->size(); ++i)
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

void save(const reach::core::ReachDatabase& db, const std::string& filename)
{
  std::ofstream ofs(filename);
  boost::archive::binary_oarchive oa(ofs);
  oa << db;
}

reach::core::ReachDatabase load(const std::string& filename)
{
  std::ifstream ifs(filename);
  boost::archive::binary_iarchive ia(ifs);
  reach::core::ReachDatabase db;
  ia >> db;
  return db;
}

}  // namespace core
}  // namespace reach
