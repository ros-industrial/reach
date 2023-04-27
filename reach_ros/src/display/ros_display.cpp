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
#include <reach_ros/display/ros_display.h>
#include <reach_ros/utils.h>

#include <reach/plugin_utils.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>

const static std::string JOINT_STATES_TOPIC = "reach_joints";
const static std::string MESH_MARKER_TOPIC = "collision_mesh";
const static std::string NEIGHBORS_MARKER_TOPIC = "reach_neighbors";
const static std::string INTERACTIVE_MARKER_TOPIC = "reach_int_markers";

namespace reach_ros
{
namespace display
{
ROSDisplay::ROSDisplay(std::string kinematic_base_frame, double marker_scale, bool use_full_color_range)
  : kinematic_base_frame_(std::move(kinematic_base_frame))
  , marker_scale_(marker_scale)
  , use_full_color_range_(use_full_color_range)
{
  utils::initROS();
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(INTERACTIVE_MARKER_TOPIC, reach_ros::utils::node);
  joint_state_pub_ = reach_ros::utils::node->create_publisher<sensor_msgs::msg::JointState>(JOINT_STATES_TOPIC, 1);
  mesh_pub_ = reach_ros::utils::node->create_publisher<visualization_msgs::msg::Marker>(MESH_MARKER_TOPIC, 1);
  neighbors_pub_ = reach_ros::utils::node->create_publisher<visualization_msgs::msg::Marker>(NEIGHBORS_MARKER_TOPIC, 1);
}

void ROSDisplay::showEnvironment() const
{
  mesh_pub_->publish(collision_marker_);
}

void ROSDisplay::updateRobotPose(const std::map<std::string, double>& pose) const
{
  sensor_msgs::msg::JointState msg;
  std::transform(pose.begin(), pose.end(), std::back_inserter(msg.name),
                 [](const std::pair<const std::string, double>& pair) { return pair.first; });
  std::transform(pose.begin(), pose.end(), std::back_inserter(msg.position),
                 [](const std::pair<const std::string, double>& pair) { return pair.second; });

  joint_state_pub_->publish(msg);
}

void ROSDisplay::showResults(const reach::ReachResult& db) const
{
  server_->clear();

  // Create a callback for when a marker is clicked on
  auto show_goal_cb = [this, db](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstPtr& fb) {
    std::size_t idx = std::strtoul(fb->marker_name.c_str(), nullptr, 10);
    updateRobotPose(db.at(idx).goal_state);
  };

  Eigen::MatrixX3f heatmap_colors = reach::computeHeatMapColors(db, use_full_color_range_);

  for (std::size_t i = 0; i < db.size(); ++i)
  {
    const std::string id = std::to_string(i);
    auto marker = utils::makeInteractiveMarker(id, db[i], kinematic_base_frame_, marker_scale_, heatmap_colors.row(i));
    server_->insert(std::move(marker));
    server_->setCallback(id, show_goal_cb);
  }

  server_->applyChanges();
}

void ROSDisplay::showReachNeighborhood(const std::map<std::size_t, reach::ReachRecord>& neighborhood) const
{
  if (!neighborhood.empty())
  {
    std::vector<geometry_msgs::msg::Point> pt_array;

    for (auto it = neighborhood.begin(); it != neighborhood.end(); ++it)
    {
      // Visualize points reached around input point
      const Eigen::Vector3d& pt = it->second.goal.translation();
      pt_array.push_back(tf2::toMsg(pt));
    }

    // Create points marker, publish it, and move robot to result state for  given point
    visualization_msgs::msg::Marker pt_marker = utils::makeMarker(pt_array, kinematic_base_frame_, marker_scale_);
    neighbors_pub_->publish(pt_marker);
  }
}

void ROSDisplay::setCollisionMarker(std::string collision_mesh_filename, const std::string collision_mesh_frame)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = collision_mesh_frame;
  marker.pose.orientation.w = 1.0;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  marker.mesh_resource = collision_mesh_filename;
  marker.mesh_use_embedded_materials = true;

  // Color
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Overwrite the internal member
  collision_marker_ = marker;
  showEnvironment();
}

reach::Display::ConstPtr ROSDisplayFactory::create(const YAML::Node& config) const
{
  auto kinematic_base_frame = reach::get<std::string>(config, "kinematic_base_frame");
  auto marker_scale = reach::get<double>(config, "marker_scale");

  // Optionally set the 'use_full_color_range' parameter
  bool use_fcr = false;
  if (config["use_full_color_range"])
    use_fcr = reach::get<bool>(config, "use_full_color_range");

  auto display = std::make_shared<ROSDisplay>(kinematic_base_frame, marker_scale, use_fcr);

  // Optionally add a collision mesh
  const std::string collision_mesh_filename_key = "collision_mesh_filename";
  const std::string collision_mesh_frame_key = "collision_mesh_key";
  if (config[collision_mesh_filename_key])
  {
    auto collision_mesh_filename = reach::get<std::string>(config, collision_mesh_filename_key);
    std::string collision_mesh_frame = config[collision_mesh_frame_key] ?
                                           reach::get<std::string>(config, collision_mesh_frame_key) :
                                           kinematic_base_frame;

    display->setCollisionMarker(collision_mesh_filename, collision_mesh_frame);
  }

  return display;
}

}  // namespace display
}  // namespace reach_ros

EXPORT_DISPLAY_PLUGIN(reach_ros::display::ROSDisplayFactory, ROSDisplay)
