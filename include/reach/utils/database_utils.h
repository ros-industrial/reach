#ifndef REACH_UTILS_DATABASE_UTILS_H
#define REACH_UTILS_DATABASE_UTILS_H

#include <robot_reach_study/ReachRecord.h>
#include <moveit/robot_state/robot_state.h>
#include <string>

namespace reach
{
namespace utils
{

/**
 *
 */
template <class T>
bool toFile(const std::string& path,
            const T& msg);

/**
 *
 */
template <class T>
bool fromFile(const std::string& path,
              T& msg);

/**
 * @brief makeRecordSuccess creates a ReachRecord message for the successful solution of the robot's IK for a given target pose
 * @param id
 * @param goal
 * @param seed_state
 * @param goal_state
 * @param score
 * @return a ReachRecord message containing information about the robot pose, to be saved in the reach database
 */
robot_reach_study::ReachRecord makeRecordSuccess(const std::string& id,
                                                 const geometry_msgs::Pose& goal,
                                                 const moveit::core::RobotState& seed_state,
                                                 const moveit::core::RobotState& goal_state,
                                                 const double score);

/**
 * @brief makeRecordFailure creates a ReachRecord message for the unsuccessful solution of the robot's IK for a given target pose
 * @param id
 * @param goal
 * @param seed_state
 * @param score
 * @return a ReachRecord message containing informatino about the robot pose, to be saved in the reach database
 */
robot_reach_study::ReachRecord makeRecordFailure(const std::string& id,
                                                 const geometry_msgs::Pose& goal,
                                                 const moveit::core::RobotState& seed_state,
                                                 const double score);

} // namespace utils
} // namespace reach

#endif // REACH_UTILS_DATABASE_UTILS_H
