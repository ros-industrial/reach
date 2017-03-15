#include <robot_reach_study/reach_database.h>
#include <robot_reach_study/param_helpers.h>
#include <robot_reach_study/ik_helper.h>
#include <moveit/robot_state/conversions.h>
#include <atomic>

namespace // private utils namespace
{

robot_reach_study::ReachDatabase
toReachDatabase(const std::unordered_map<std::string, robot_reach_study::ReachRecord>& db)
{
  robot_reach_study::ReachDatabase msg;
  for (const auto& record : db)
  {
    msg.records.push_back(record.second);
  }
  return msg;
}

} // end private namespace

void robot_reach_study::Database::save(const std::string &filename) const
{
  std::lock_guard<std::mutex> lock {mutex_};
  robot_reach_study::ReachDatabase msg = toReachDatabase(map_);

  msg.avg_score = total_score_;
  msg.norm_avg_score = norm_total_score_;
  msg.reach_percentage = reach_percentage_;
  msg.avg_neighbors = avg_neighbors_;
  msg.avg_joint_distance = avg_joint_distance_;

  if (!toFile(filename, msg))
  {
    throw std::runtime_error("Unable to save database to file: " + filename);
  }
}

bool robot_reach_study::Database::load(const std::string &filename)
{
  robot_reach_study::ReachDatabase msg;
  if (!fromFile(filename, msg))
  {
    return false;
  }

  std::lock_guard<std::mutex> lock {mutex_};

  for (const auto& r : msg.records)
  {
    putHelper(r);
    total_score_ = msg.avg_score;
    norm_total_score_ = msg.norm_avg_score;
    avg_neighbors_ = msg.avg_neighbors;
    reach_percentage_ = msg.reach_percentage;
    avg_joint_distance_ = msg.avg_joint_distance;
  }
  return true;
}

boost::optional<robot_reach_study::ReachRecord> robot_reach_study::Database::get(const std::string &id) const
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

void robot_reach_study::Database::put(const robot_reach_study::ReachRecord &record)
{
  std::lock_guard<std::mutex> lock {mutex_};
  return putHelper(record);
}

void robot_reach_study::Database::putHelper(const robot_reach_study::ReachRecord &record)
{
  map_[record.id] = record;
}

int robot_reach_study::Database::count()
{
  int count = 0;
  for(auto it = this->begin(); it != this->end(); ++it) {++count;}
  return count;
}

void robot_reach_study::Database::calculateResults()
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

  reach_percentage_ = 100.0 * pct_success;
  total_score_ = score;
  norm_total_score_ = score / pct_success;

  printResults();
}

void robot_reach_study::Database::printResults()
{
  ROS_INFO("------------------------------------------------");
  ROS_INFO("Percent Reached = %f", reach_percentage_);
  ROS_INFO("Total points score = %f", total_score_);
  ROS_INFO("Normalized total points score = %f", norm_total_score_);
  ROS_INFO("Average reachable neighbors = %f", avg_neighbors_);
  ROS_INFO("Average joint distance = %f", avg_joint_distance_);
  ROS_INFO("------------------------------------------------");
}

robot_reach_study::ReachRecord robot_reach_study::makeRecordSuccess(const std::string &id,
                                                                    const geometry_msgs::Pose &goal,
                                                                    const moveit::core::RobotState &seed_state,
                                                                    const moveit::core::RobotState &goal_state,
                                                                    const double score)
{
  robot_reach_study::ReachRecord r;
  r.id = id;
  r.goal = goal;
  r.reached = true;
  moveit::core::robotStateToRobotStateMsg(seed_state, r.seed_state);
  moveit::core::robotStateToRobotStateMsg(goal_state, r.goal_state);
  r.score = score;
  return r;
}

robot_reach_study::ReachRecord robot_reach_study::makeRecordFailure(const std::string &id,
                                                                    const geometry_msgs::Pose &goal,
                                                                    const moveit::core::RobotState &seed_state,
                                                                    const double score)
{
  robot_reach_study::ReachRecord r;
  r.id = id;
  r.goal = goal;
  r.reached = false;
  moveit::core::robotStateToRobotStateMsg(seed_state, r.seed_state);
  moveit::core::robotStateToRobotStateMsg(seed_state, r.goal_state);
  r.score = score;
  return r;
}
