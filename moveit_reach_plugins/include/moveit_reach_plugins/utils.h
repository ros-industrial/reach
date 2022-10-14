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
#ifndef MOVEIT_REACH_PLUGINS_KINEMATICS_UTILS_H
#define MOVEIT_REACH_PLUGINS_KINEMATICS_UTILS_H

#include <Eigen/Dense>
#include <string>
#include <moveit_msgs/CollisionObject.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>

namespace reach
{
class ReachRecord;
}

namespace moveit_reach_plugins
{
namespace utils
{
moveit_msgs::CollisionObject createCollisionObject(const std::string& mesh_filename, const std::string& parent_link,
                                                   const std::string& object_name);

visualization_msgs::Marker makeVisual(const reach::ReachRecord& r, const std::string& frame, const double scale,
                                      const std::string& ns = "reach",
                                      const Eigen::Vector3f& color = { 0.5, 0.5, 0.5 });

visualization_msgs::InteractiveMarker makeInteractiveMarker(const reach::ReachRecord& r, const std::string& frame,
                                                            const double scale,
                                                            const Eigen::Vector3f& rgb_color = { 0.5, 0.5, 0.5 });

visualization_msgs::Marker makeMarker(const std::vector<geometry_msgs::Point>& pts, const std::string& frame,
                                      const double scale, const std::string& ns = "");

std::vector<double> transcribeInputMap(const std::map<std::string, double>& input,
                                       const std::vector<std::string>& joint_names);

}  // namespace utils
}  // namespace moveit_reach_plugins

#endif  // MOVEIT_REACH_PLUGINS_KINEMATICS_UTILS_H
