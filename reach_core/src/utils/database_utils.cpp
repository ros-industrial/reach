#include "reach/utils/database_utils.h"
#include <moveit/robot_state/conversions.h>

namespace reach
{
namespace utils
{

reach_msgs::ReachRecord makeRecord(const std::string &id,
                                   const bool reached,
                                   const geometry_msgs::Pose &goal,
                                   const moveit::core::RobotState &seed_state,
                                   const moveit::core::RobotState &goal_state,
                                   const double score)
{
  reach_msgs::ReachRecord r;
  r.id = id;
  r.goal = goal;
  r.reached = reached;
  moveit::core::robotStateToRobotStateMsg(seed_state, r.seed_state);
  moveit::core::robotStateToRobotStateMsg(goal_state, r.goal_state);
  r.score = score;
  return r;
}

} // namespace utils
} // namespace reach

