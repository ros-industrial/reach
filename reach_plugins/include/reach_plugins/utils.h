#ifndef REACH_PLUGINS_KINEMATICS_UTILS_H
#define REACH_PLUGINS_KINEMATICS_UTILS_H

#include <string>
#include <moveit_msgs/CollisionObject.h>
#include <reach_msgs/ReachRecord.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>

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

/**
 * @brief makeInteractiveMarker
 * @param r
 * @param frame
 * @param scale
 * @return
 */
visualization_msgs::Marker makeVisual(const reach_msgs::ReachRecord& r,
                                      const std::string& frame,
                                      const double scale,
                                      const std::string& ns = "reach",
                                      const boost::optional<std::vector<float>>& color = {});

/**
 * @brief makeInteractiveMarker
 * @param r
 * @param frame
 * @param scale
 * @return
 */
visualization_msgs::InteractiveMarker makeInteractiveMarker(const reach_msgs::ReachRecord& r,
                                                            const std::string& frame,
                                                            const double scale);

} // namespace utils
} // namespace reach_plugins

#endif // REACH_PLUGINS_KINEMATICS_UTILS_H
