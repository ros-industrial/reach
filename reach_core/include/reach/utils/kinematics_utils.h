#ifndef REACH_UTILS_KINEMATICS_UTILS_H
#define REACH_UTILS_KINEMATICS_UTILS_H

#include <Eigen/Dense>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/CollisionObject.h>

namespace reach
{
namespace utils
{

/**
 * @brief createFrame
 * @param pt
 * @param norm
 * @return
 */
Eigen::Affine3d createFrame(const Eigen::Vector3f& pt,
                            const Eigen::Vector3f& norm);

/**
 * @brief createCollisionObject
 * @param mesh_filename
 * @param parent_link
 * @param touch_links
 * @param object_name
 * @return
 */
moveit_msgs::CollisionObject createCollisionObject(const std::string& mesh_filename,
                                                   const std::string& parent_link,
                                                   const std::vector<std::string>& touch_links,
                                                   const std::string& object_name);

/**
 * @brief getManipulability
 * @param state
 * @param jmg
 * @return
 */
double getManipulability(const moveit::core::RobotState& state,
                         const moveit::core::JointModelGroup* jmg);

/**
 * @brief getJointPenalty
 * @param state
 * @param jmg
 * @param joint_limits
 * @return
 */
double getJointPenalty(const moveit::core::RobotState& state,
                       const moveit::core::JointModelGroup* jmg,
                       std::vector<std::vector<double>>& joint_limits);

} // namespace utils
} // namespace reach

#endif // REACH_UTILS_KINEMATICS_UTILS_H
