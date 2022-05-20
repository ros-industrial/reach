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
#include "reach_core/utils/visualization_utils.h"

#include "tf2_eigen/tf2_eigen.h"

#include <pcl/features/moment_of_inertia_estimation.h>

const static double ARROW_SCALE_RATIO = 6.0;
const static double NEIGHBOR_MARKER_SCALE_RATIO = ARROW_SCALE_RATIO / 2.0;
const static double SPHERE_DIAMETER = 0.005;

namespace reach {
namespace utils {

visualization_msgs::msg::Marker makeVisual(
    const rclcpp::Node::SharedPtr &node, const reach_msgs::msg::ReachRecord &r,
    const std::string &frame, const double scale, const std::string &ns,
    const boost::optional<std::vector<float>> &color) {
  static int idx = 0;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = node->now();
  marker.ns = ns;
  marker.id = idx++;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  Eigen::Isometry3d goal_eigen;
  tf2::fromMsg(r.goal, goal_eigen);

  // Transform arrow such that arrow x-axis points along goal pose z-axis (Rviz
  // convention) convert msg parameter goal to Eigen matrix
  Eigen::AngleAxisd rot_flip_normal(M_PI, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rot_x_to_z(-M_PI / 2, Eigen::Vector3d::UnitY());

  // Transform
  goal_eigen = goal_eigen * rot_flip_normal * rot_x_to_z;

  // Convert back to geometry_msgs pose
  geometry_msgs::msg::Pose msg;
  msg = tf2::toMsg(goal_eigen);
  marker.pose = msg;

  marker.scale.x = scale;
  marker.scale.y = scale / ARROW_SCALE_RATIO;
  marker.scale.z = scale / ARROW_SCALE_RATIO;

  if (color) {
    std::vector<float> color_vec = *color;
    marker.color.r = color_vec[0];
    marker.color.g = color_vec[1];
    marker.color.b = color_vec[2];
    marker.color.a = color_vec[3];
  } else {
    marker.color.a = 1.0;  // Don't forget to set the alpha!

    if (r.reached) {
      marker.color.r = 1.0 - r.retrieved_fraction;
      marker.color.g = 1.0 - r.retrieved_fraction;
      marker.color.b = r.retrieved_fraction;
    } else {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }
  }

  return marker;
}

visualization_msgs::msg::Marker makeVisualTraj(
    const rclcpp::Node::SharedPtr &node, const reach_msgs::msg::ReachRecord &r,
    const std::string &frame, const double scale, const std::string &ns,
    const boost::optional<std::vector<float>> &color) {
  static int idx = 0;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = node->now();
  marker.ns = ns;
  marker.id = idx++;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Convert back to geometry_msgs pose
  geometry_msgs::msg::Pose msg;
  marker.pose = msg;

  size_t wpts_size = r.waypoints.size();
  // fill the points
  for (size_t i = 0; i < wpts_size && ((wpts_size % 7u) == 0u); i += 7u) {
    geometry_msgs::msg::Point p_tmp;
    p_tmp.set__x(r.waypoints[i + 0]);
    p_tmp.set__y(r.waypoints[i + 1]);
    p_tmp.set__z(r.waypoints[i + 2]);
    marker.points.push_back(p_tmp);
  }

  // sphere diameters
  marker.scale.x = SPHERE_DIAMETER;
  marker.scale.y = SPHERE_DIAMETER;
  marker.scale.z = SPHERE_DIAMETER;

  if (color) {
    std::vector<float> color_vec = *color;
    marker.color.r = color_vec[0];
    marker.color.g = color_vec[1];
    marker.color.b = color_vec[2];
    marker.color.a = color_vec[3];
  } else {
    marker.color.a = 1.0;  // Don't forget to set the alpha!

    if (r.reached) {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }
  }

  return marker;
}

visualization_msgs::msg::InteractiveMarker makeInteractiveMarker(
    const rclcpp::Node::SharedPtr &node, const reach_msgs::msg::ReachRecord &r,
    const std::string &frame, const double scale) {
  visualization_msgs::msg::InteractiveMarker m;
  m.header.frame_id = frame;
  m.name = r.id;

  // Control
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  // Visuals
  auto visual = utils::makeVisual(node, r, frame, scale);
  control.markers.push_back(visual);
  // TODO (livanov93): what to do with this since trajectory fraction now is
  // colorized by scaling between red and blue
  //  if (!r.waypoints.empty()) {
  //    auto visual_traj = utils::makeVisualTraj(node, r, frame, scale);
  //    control.markers.push_back(visual_traj);
  //  }
  m.controls.push_back(control);

  return m;
}

visualization_msgs::msg::Marker makeMarker(
    const rclcpp::Node::SharedPtr &node,
    const std::vector<geometry_msgs::msg::Point> &pts, const std::string &frame,
    const double scale, const std::string &ns) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = node->now();
  marker.ns = ns;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.scale.x = marker.scale.y = marker.scale.z =
      scale / NEIGHBOR_MARKER_SCALE_RATIO;

  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = 0;
  marker.color.g = 1.0;
  marker.color.b = 0;

  for (std::size_t i = 0; i < pts.size(); ++i) {
    marker.points.push_back(pts[i]);
  }

  return marker;
}

double getMajorLength(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
  pcl::MomentOfInertiaEstimation<pcl::PointNormal> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.compute();

  pcl::PointNormal min_pt, max_pt, position;
  Eigen::Matrix3f rotation;
  feature_extractor.getOBB(min_pt, max_pt, position, rotation);

  std::vector<double> lengths;
  lengths.push_back(max_pt.x - min_pt.x);
  lengths.push_back(max_pt.y - min_pt.y);
  lengths.push_back(max_pt.z - min_pt.z);

  return *(std::max_element(lengths.begin(), lengths.end()));
}

}  // namespace utils
}  // namespace reach
