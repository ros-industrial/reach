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
#ifndef REACH_UTILS_VISUALIZATION_UTILS_H
#define REACH_UTILS_VISUALIZATION_UTILS_H

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <reach_msgs/msg/reach_record.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>

namespace reach {
namespace utils {

/**
 * @brief makeInteractiveMarker
 * @param r
 * @param frame
 * @param scale
 * @return
 */
visualization_msgs::msg::Marker makeVisual(
    const rclcpp::Node::SharedPtr &node, const reach_msgs::msg::ReachRecord &r,
    const std::string &frame, const double scale,
    const std::string &ns = "reach",
    const boost::optional<std::vector<float>> &color = {});

/**
 * @brief makeInteractiveMarker
 * @param r
 * @param frame
 * @param scale
 * @return
 */
visualization_msgs::msg::Marker makeVisualTraj(
    const rclcpp::Node::SharedPtr &node, const reach_msgs::msg::ReachRecord &r,
    const std::string &frame, const double scale,
    const std::string &ns = "reach",
    const boost::optional<std::vector<float>> &color = {});

/**
 * @brief makeInteractiveMarker
 * @param r
 * @param frame
 * @param scale
 * @return
 */
visualization_msgs::msg::InteractiveMarker makeInteractiveMarker(
    const rclcpp::Node::SharedPtr &node, const reach_msgs::msg::ReachRecord &r,
    const std::string &frame, const double scale);

/**
 * @brief makeMarker
 * @param pts
 * @param frame
 * @param scale
 * @param ns
 * @return
 */
visualization_msgs::msg::Marker makeMarker(
    const rclcpp::Node::SharedPtr &node,
    const std::vector<geometry_msgs::msg::Point> &pts, const std::string &frame,
    const double scale, const std::string &ns = "");

/**
 * @brief getMajorLength
 * @param cloud
 * @return
 */
double getMajorLength(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);

}  // namespace utils
}  // namespace reach

#endif  // REACH_UTILS_VISUALIZATION_UTILS_H
