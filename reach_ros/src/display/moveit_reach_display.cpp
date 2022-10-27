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
#include <reach_ros/display/moveit_reach_display.h>
#include <reach_ros/utils.h>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <pcl/point_types_conversion.h>
#include <reach_core/plugin_utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <yaml-cpp/yaml.h>

const static std::string PLANNING_SCENE_TOPIC = "planning_scene_display";
const static std::string MARKER_TOPIC = "reach_neighbors";
const static std::string INTERACTIVE_MARKER_TOPIC = "reach_int_markers";

namespace reach_ros
{
namespace display
{
MoveItReachDisplay::MoveItReachDisplay(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                                       std::string collision_mesh_filename, double marker_scale)
  : model_(model)
  , jmg_(model_->getJointModelGroup(planning_group))
  , collision_mesh_filename_(std::move(collision_mesh_filename))
  , marker_scale_(marker_scale)
  , scene_(new planning_scene::PlanningScene(model_))
  , server_(INTERACTIVE_MARKER_TOPIC)
{
  // Add the collision object to the planning scene
  const std::string object_name = "reach_object";
  moveit_msgs::CollisionObject obj =
      utils::createCollisionObject(collision_mesh_filename_, jmg_->getSolverInstance()->getBaseFrame(), object_name);
  if (!scene_->processCollisionObjectMsg(obj))
    throw std::runtime_error("Failed to add collision mesh to planning scene");

  scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>(PLANNING_SCENE_TOPIC, 1, true);
  neighbors_pub_ = nh_.advertise<visualization_msgs::Marker>(MARKER_TOPIC, 1, true);
}

void MoveItReachDisplay::showEnvironment() const
{
  moveit_msgs::PlanningScene scene_msg;
  scene_->getPlanningSceneMsg(scene_msg);
  scene_pub_.publish(scene_msg);
}

void MoveItReachDisplay::updateRobotPose(const std::map<std::string, double>& pose) const
{
  std::vector<std::string> joint_names = jmg_->getActiveJointModelNames();
  std::vector<double> joints = utils::transcribeInputMap(pose, joint_names);

  moveit_msgs::PlanningScene scene_msg;
  scene_msg.is_diff = true;
  scene_msg.robot_state.is_diff = true;
  scene_msg.robot_state.joint_state.name = joint_names;
  scene_msg.robot_state.joint_state.position = joints;
  scene_pub_.publish(scene_msg);
}

void MoveItReachDisplay::showResults(const reach::ReachDatabase& db) const
{
  server_.clear();

  // Create a callback for when a marker is clicked on
  auto show_goal_cb = [this, &db](const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb) {
    updateRobotPose(db.get(fb->marker_name).goal_state);
  };

  // Find the max of the scores
  double max_score = 0;
  for (auto it = db.begin(); it != db.end(); ++it)
  {
    if (it->second.score > max_score)
    {
      max_score = it->second.score;
    }
  }

  for (auto it = db.begin(); it != db.end(); ++it)
  {
    // Compute the color of the marker as a heatmap from blue to red using HSV space
    const float max_h = 0.75f * 360.0f;  // Corresponds to blue color
    const float h = max_h - (static_cast<float>(it->second.score / max_score) * max_h);
    const float s = 1.0f;
    const float v = it->second.reached ? 1.0f : 0.0f;

    // Convert to RGB
    const pcl::PointXYZHSV pt_hsv(h, s, v);
    pcl::PointXYZRGB pt_rgb;
    pcl::PointXYZHSVtoXYZRGB(pt_hsv, pt_rgb);
    const Eigen::Vector3f rgb_color = pt_rgb.getRGBVector3i().cast<float>() / 255.0f;

    auto marker =
        utils::makeInteractiveMarker(it->second, jmg_->getSolverInstance()->getBaseFrame(), marker_scale_, rgb_color);
    server_.insert(std::move(marker));
    server_.setCallback(it->second.id, show_goal_cb);
  }

  server_.applyChanges();
}

void MoveItReachDisplay::showReachNeighborhood(const std::vector<reach::ReachRecord>& neighborhood) const
{
  if (!neighborhood.empty())
  {
    std::vector<geometry_msgs::Point> pt_array;

    for (const reach::ReachRecord& rec : neighborhood)
    {
      // Visualize points reached around input point
      const Eigen::Vector3d& pt = rec.goal.translation();
      pt_array.push_back(tf2::toMsg(pt));
    }

    // Create points marker, publish it, and move robot to result state for  given point
    visualization_msgs::Marker pt_marker =
        utils::makeMarker(pt_array, jmg_->getSolverInstance()->getBaseFrame(), marker_scale_);
    neighbors_pub_.publish(pt_marker);
  }
}

reach::Display::ConstPtr MoveItReachDisplayFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  auto collision_mesh_filename = reach::get<std::string>(config, "collision_mesh_filename");
  auto marker_scale = reach::get<double>(config, "marker_scale");

  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return std::make_shared<MoveItReachDisplay>(model, planning_group, collision_mesh_filename, marker_scale);
}

}  // namespace display
}  // namespace reach_ros

EXPORT_DISPLAY_PLUGIN(reach_ros::display::MoveItReachDisplayFactory, MoveItReachDisplay)
