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
#include <sensor_msgs/JointState.h>
#include <tf2_eigen/tf2_eigen.h>
#include <yaml-cpp/yaml.h>

const static std::string JOINT_STATES_TOPIC = "reach_joints";
const static std::string MESH_MARKER_TOPIC = "collision_mesh";
const static std::string NEIGHBORS_MARKER_TOPIC = "reach_neighbors";
const static std::string INTERACTIVE_MARKER_TOPIC = "reach_int_markers";

namespace reach_ros
{
namespace display
{
ROSDisplay::ROSDisplay(std::string kinematic_base_frame, double marker_scale)
  : kinematic_base_frame_(std::move(kinematic_base_frame))
  , marker_scale_(marker_scale)
  , server_(INTERACTIVE_MARKER_TOPIC)
{
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(JOINT_STATES_TOPIC, 1, true);
  mesh_pub_ = nh_.advertise<visualization_msgs::Marker>(MESH_MARKER_TOPIC, 1, true);
  neighbors_pub_ = nh_.advertise<visualization_msgs::Marker>(NEIGHBORS_MARKER_TOPIC, 1, true);
}

void ROSDisplay::showEnvironment() const
{
  mesh_pub_.publish(collision_marker_);
}

void ROSDisplay::updateRobotPose(const std::map<std::string, double>& pose) const
{
  sensor_msgs::JointState msg;
  std::transform(pose.begin(), pose.end(), std::back_inserter(msg.name),
                 [](const std::pair<const std::string, double>& pair) { return pair.first; });
  std::transform(pose.begin(), pose.end(), std::back_inserter(msg.position),
                 [](const std::pair<const std::string, double>& pair) { return pair.second; });

  joint_state_pub_.publish(msg);
}

void ROSDisplay::showResults(const reach::ReachResult& db) const
{
  server_.clear();

  // Create a callback for when a marker is clicked on
  auto show_goal_cb = [this, db](const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb) {
    std::size_t idx = std::strtoul(fb->marker_name.c_str(), nullptr, 10);
    updateRobotPose(db.at(idx).goal_state);
  };

  Eigen::MatrixX3f heatmap_colors = reach::computeHeatMapColors(db);

  for (std::size_t i = 0; i < db.size(); ++i)
  {
    const std::string id = std::to_string(i);
    auto marker = utils::makeInteractiveMarker(id, db[i], kinematic_base_frame_, marker_scale_, heatmap_colors.row(i));
    server_.insert(std::move(marker));
    server_.setCallback(id, show_goal_cb);
  }

  server_.applyChanges();
}

void ROSDisplay::showReachNeighborhood(const std::map<std::size_t, reach::ReachRecord>& neighborhood) const
{
  if (!neighborhood.empty())
  {
    std::vector<geometry_msgs::Point> pt_array;

    for (auto it = neighborhood.begin(); it != neighborhood.end(); ++it)
    {
      // Visualize points reached around input point
      const Eigen::Vector3d& pt = it->second.goal.translation();
      pt_array.push_back(tf2::toMsg(pt));
    }

    // Create points marker, publish it, and move robot to result state for  given point
    visualization_msgs::Marker pt_marker = utils::makeMarker(pt_array, kinematic_base_frame_, marker_scale_);
    neighbors_pub_.publish(pt_marker);
  }
}

void ROSDisplay::setCollisionMarker(std::string collision_mesh_filename, const std::string collision_mesh_frame)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = collision_mesh_frame;
  marker.pose.orientation.w = 1.0;
  marker.action = visualization_msgs::Marker::ADD;

  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
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

  auto display = std::make_shared<ROSDisplay>(kinematic_base_frame, marker_scale);

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
