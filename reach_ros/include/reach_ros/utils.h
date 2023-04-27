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
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <map>

namespace reach
{
class ReachRecord;
}

namespace reach_ros
{
namespace utils
{
moveit_msgs::msg::CollisionObject createCollisionObject(const std::string& mesh_filename, const std::string& parent_link,
                                                   const std::string& object_name);

visualization_msgs::msg::Marker makeVisual(const reach::ReachRecord& r, const std::string& frame, const double scale,
                                      const std::string& ns = "reach",
                                      const Eigen::Vector3f& color = { 0.5, 0.5, 0.5 });

visualization_msgs::msg::InteractiveMarker makeInteractiveMarker(const std::string& id, const reach::ReachRecord& r,
                                                            const std::string& frame, const double scale,
                                                            const Eigen::Vector3f& rgb_color = { 0.5, 0.5, 0.5 });

visualization_msgs::msg::Marker makeMarker(const std::vector<geometry_msgs::msg::Point>& pts, const std::string& frame,
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
 * Note: this function first checks if ROS has already been initialized before calling rclcpp::init
 */
bool ros_initialized=false;
rclcpp::Node::SharedPtr node;
rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
std::shared_ptr<std::thread> executor_thread;
void initROS(const std::string& node_name = "reach_study_plugin_node")
{
  if (!ros_initialized)
  {
    ros_initialized=true;
    int argc = 0;
    rclcpp::init(argc, nullptr);
    node = std::make_shared<rclcpp::Node>("reach_study_node");
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor_thread = std::make_shared<std::thread>(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, *&executor));
  }
}

}  // namespace utils
}  // namespace reach_ros

#endif  // REACH_ROS_KINEMATICS_UTILS_H
