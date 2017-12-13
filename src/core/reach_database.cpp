#include <reach/core/reach_database.h>
#include <reach/utils/database_utils.h>

namespace // private utils namespace
{

robot_reach_study::ReachDatabase
toReachDatabase(const std::unordered_map<std::string,
                robot_reach_study::ReachRecord>& db)
{
  robot_reach_study::ReachDatabase msg;
  for (const auto& record : db)
  {
    msg.records.push_back(record.second);
  }
  return msg;
}

} // end private namespace

namespace reach
{
namespace core
{

void ReachDatabase::save(const std::string &filename) const
{
  std::lock_guard<std::mutex> lock {mutex_};
  robot_reach_study::ReachDatabase msg = toReachDatabase(map_);

  msg.total_pose_score = results_.total_pose_score_;
  msg.norm_total_pose_score = results_.norm_total_pose_score_;
  msg.reach_percentage = results_.reach_percentage_;
  msg.avg_num_neighbors = results_.avg_num_neighbors_;
  msg.avg_joint_distance = results_.avg_joint_distance_;

  if (!utils::toFile(filename, msg))
  {
    throw std::runtime_error("Unable to save database to file: " + filename);
  }
}

bool ReachDatabase::load(const std::string &filename)
{
  robot_reach_study::ReachDatabase msg;
  if (!utils::fromFile(filename, msg))
  {
    return false;
  }

  std::lock_guard<std::mutex> lock {mutex_};

  for (const auto& r : msg.records)
  {
    putHelper(r);
    results_.reach_percentage_ = msg.reach_percentage;
    results_.total_pose_score_ = msg.total_pose_score;
    results_.norm_total_pose_score_ = msg.norm_total_pose_score;
    results_.avg_num_neighbors_ = msg.avg_num_neighbors;
    results_.avg_joint_distance_ = msg.avg_joint_distance;
  }
  return true;
}

boost::optional<robot_reach_study::ReachRecord> ReachDatabase::get(const std::string &id) const
{
  std::lock_guard<std::mutex> lock {mutex_};
  auto it = map_.find(id);
  if (it != map_.end())
  {
    return {it->second};
  }
  else
  {
    return {};
  }
}

void ReachDatabase::put(const robot_reach_study::ReachRecord &record)
{
  std::lock_guard<std::mutex> lock {mutex_};
  return putHelper(record);
}

void ReachDatabase::putHelper(const robot_reach_study::ReachRecord &record)
{
  map_[record.id] = record;
}

int ReachDatabase::count()
{
  int count = 0;
  for(auto it = this->begin(); it != this->end(); ++it) {++count;}
  return count;
}

void ReachDatabase::calculateResults()
{
  unsigned int success = 0, total = 0;
  double score = 0.0;
  for(int i = 0; i < this->count(); ++i)
  {
    robot_reach_study::ReachRecord msg = *this->get(std::to_string(i));

    if(msg.reached)
    {
      success++;
      score += msg.score;
    }

    total ++;
  }
  const float pct_success = static_cast<float>(success) / static_cast<float>(total);

  results_.reach_percentage_ = 100.0 * pct_success;
  results_.total_pose_score_ = score;
  results_.norm_total_pose_score_ = score / pct_success;
}

void ReachDatabase::printResults()
{
  ROS_INFO("------------------------------------------------");
  ROS_INFO("Percent Reached = %f", results_.reach_percentage_);
  ROS_INFO("Total points score = %f", results_.total_pose_score_);
  ROS_INFO("Normalized total points score = %f", results_.norm_total_pose_score_);
  ROS_INFO("Average reachable neighbors = %f", results_.avg_num_neighbors_);
  ROS_INFO("Average joint distance = %f", results_.avg_joint_distance_);
  ROS_INFO("------------------------------------------------");
}

} // namespace core
} // namespace reach

