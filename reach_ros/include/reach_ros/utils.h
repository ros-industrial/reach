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
#ifndef REACH_ROS_KINEMATICS_UTILS_H
#define REACH_ROS_KINEMATICS_UTILS_H

#include <Eigen/Dense>
#include <string>
#include <moveit_msgs/CollisionObject.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>

namespace reach
{
class ReachRecord;
}

namespace reach_ros
{
namespace utils
{
moveit_msgs::CollisionObject createCollisionObject(const std::string& mesh_filename, const std::string& parent_link,
                                                   const std::string& object_name);

visualization_msgs::Marker makeVisual(const reach::ReachRecord& r, const std::string& frame, const double scale,
                                      const std::string& ns = "reach",
                                      const Eigen::Vector3f& color = { 0.5, 0.5, 0.5 });

visualization_msgs::InteractiveMarker makeInteractiveMarker(const std::string& id, const reach::ReachRecord& r,
                                                            const std::string& frame, const double scale,
                                                            const Eigen::Vector3f& rgb_color = { 0.5, 0.5, 0.5 });

visualization_msgs::Marker makeMarker(const std::vector<geometry_msgs::Point>& pts, const std::string& frame,
                                      const double scale, const std::string& ns = "");

std::vector<double> transcribeInputMap(const std::map<std::string, double>& input,
                                       const std::vector<std::string>& joint_names);

/**
 * @brief Conditionally initializes ROS using an arbitary node name
 * @details In the case that ROS-enabled plugins are created and invoked in a non-ROS enabled process, ROS must be
 * initialized for the plugins to access the ROS parameter server and publish data. This function should be invoked in a
 * plugin factory or interface before attempting to utilize ROS components such that the process creating the plugin
 * factory or interface becomes a ROS node
 *
 * Note: this function first checks if ROS has already been initialized before calling ros::init
 */
void initROS(const std::string& node_name = "reach_study_plugin_node");

}  // namespace utils
}  // namespace reach_ros

#endif  // REACH_ROS_KINEMATICS_UTILS_H
