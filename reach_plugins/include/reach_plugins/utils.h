#ifndef REACH_PLUGINS_KINEMATICS_UTILS_H
#define REACH_PLUGINS_KINEMATICS_UTILS_H

#include <string>
#include <moveit_msgs/CollisionObject.h>

namespace reach_plugins
{
namespace utils
{

/**
 * @brief createCollisionObject
 * @param mesh_filename
 * @param parent_link
 * @param object_name
 * @return
 */
moveit_msgs::CollisionObject createCollisionObject(const std::string& mesh_filename,
                                                   const std::string& parent_link,
                                                   const std::string& object_name);

} // namespace utils
} // namespace reach_plugins

#endif // REACH_PLUGINS_KINEMATICS_UTILS_H
