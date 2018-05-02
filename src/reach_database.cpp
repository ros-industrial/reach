//#include <atomic>
#include <moveit/robot_state/conversions.h>
//#include <robot_reach_study/param_helpers.h>
#include <robot_reach_study/ik_helper.h>
#include <robot_reach_study/reach_database.h>
#include <robot_reach_study/utils.h>

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

namespace robot_reach_study
{

void Database::save(const std::string &filename) const
{
  std::lock_guard<std::mutex> lock {mutex_};
  robot_reach_study::ReachDatabase msg = toReachDatabase(map_);

  msg.total_pose_score = total_pose_score_;
  msg.norm_total_pose_score = norm_total_pose_score_;
  msg.reach_percentage = reach_percentage_;
  msg.avg_num_neighbors = avg_num_neighbors_;
  msg.avg_joint_distance = avg_joint_distance_;

  if (!utils::toFile(filename, msg))
  {
    throw std::runtime_error("Unable to save database to file: " + filename);
  }
}

bool Database::load(const std::string &filename)
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
    reach_percentage_ = msg.reach_percentage;
    total_pose_score_ = msg.total_pose_score;
    norm_total_pose_score_ = msg.norm_total_pose_score;
    avg_num_neighbors_ = msg.avg_num_neighbors;
    avg_joint_distance_ = msg.avg_joint_distance;
  }
  return true;
}

boost::optional<ReachRecord> Database::get(const std::string &id) const
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

void Database::put(const ReachRecord &record)
{
  std::lock_guard<std::mutex> lock {mutex_};
  return putHelper(record);
}

void Database::putHelper(const ReachRecord &record)
{
  map_[record.id] = record;
}

int Database::count()
{
  int count = 0;
  for(auto it = this->begin(); it != this->end(); ++it) {++count;}
  return count;
}

void Database::calculateResults()
{
  unsigned int success = 0, total = 0;
  double score = 0.0;
  for(int i = 0; i < this->count(); ++i)
  {
    ReachRecord msg = *this->get(std::to_string(i));

    if(msg.reached)
    {
      success++;
      score += msg.score;
    }

    total ++;
  }
  const float pct_success = static_cast<float>(success) / static_cast<float>(total);

  reach_percentage_ = 100.0 * pct_success;
  total_pose_score_ = score;
  norm_total_pose_score_ = score / pct_success;
}

void Database::printResults()
{
  ROS_INFO("------------------------------------------------");
  ROS_INFO("Percent Reached = %f", reach_percentage_);
  ROS_INFO("Total points score = %f", total_pose_score_);
  ROS_INFO("Normalized total points score = %f", norm_total_pose_score_);
  ROS_INFO("Average reachable neighbors = %f", avg_num_neighbors_);
  ROS_INFO("Average joint distance = %f", avg_joint_distance_);
  ROS_INFO("------------------------------------------------");
}

ReachRecord makeRecord(const std::string& id,
                       const bool reached,
                       const geometry_msgs::Pose& goal,
                       const moveit::core::RobotState& seed_state,
                       const moveit::core::RobotState& goal_state,
                       const double score)
{
  ReachRecord r;
  r.id = id;
  r.goal = goal;
  r.reached = reached;
  moveit::core::robotStateToRobotStateMsg(seed_state, r.seed_state);
  moveit::core::robotStateToRobotStateMsg(goal_state, r.goal_state);
  r.score = score;
  return r;
}

ReachRecord makeRecordSuccess(const std::string &id,
                              const geometry_msgs::Pose &goal,
                              const moveit::core::RobotState &seed_state,
                              const moveit::core::RobotState &goal_state,
                              const double score)
{
  ReachRecord r;
  r.id = id;
  r.goal = goal;
  r.reached = true;
  moveit::core::robotStateToRobotStateMsg(seed_state, r.seed_state);
  moveit::core::robotStateToRobotStateMsg(goal_state, r.goal_state);
  r.score = score;
  return r;
}

ReachRecord makeRecordFailure(const std::string &id,
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

} // namespace robot_reach_study

