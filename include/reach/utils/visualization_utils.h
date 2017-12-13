#ifndef REACH_UTILS_VISUALIZATION_UTILS_H
#define REACH_UTILS_VISUALIZATION_UTILS_H

#include <robot_reach_study/ReachRecord.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/InteractiveMarker.h>

namespace reach
{
namespace utils
{

/**
 * @brief makeInteractiveMarker
 * @param r
 * @param frame
 * @param scale
 * @return
 */
visualization_msgs::Marker
makeVisual(const robot_reach_study::ReachRecord& r,
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
visualization_msgs::InteractiveMarker
makeInteractiveMarker(const robot_reach_study::ReachRecord& r,
                      const std::string& frame,
                      const double scale);

/**
 * @brief makeMarker
 * @param pts
 * @param frame
 * @param scale
 * @param ns
 * @return
 */
visualization_msgs::Marker
makeMarker(const std::vector<geometry_msgs::Point>& pts,
           const std::string& frame,
           const double scale,
           const std::string& ns = "");

/**
 * @brief getMajorLength
 * @param cloud
 * @return
 */
double getMajorLength(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);

} // namespace utils
} // namespace reach

#endif // REACH_UTILS_VISUALIZATION_UTILS_H
