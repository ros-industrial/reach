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
#include "moveit_reach_plugins/display/moveit_reach_display.h"

#include "moveit_reach_plugins/utils.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene/planning_scene.h>
// conversions
#include "tf2_eigen/tf2_eigen.h"

#include <moveit/robot_state/conversions.h>

const static std::string PLANNING_SCENE_TOPIC = "planning_scene_display";
const static std::string TRAJECTORY_SCENE_TOPIC = "trajectory_display";

namespace moveit_reach_plugins {
namespace display {

MoveItReachDisplay::MoveItReachDisplay() : reach::plugins::DisplayBase() {}

bool MoveItReachDisplay::initialize(
    std::string& name, rclcpp::Node::SharedPtr node,
    const std::shared_ptr<const moveit::core::RobotModel> model) {
  RCLCPP_INFO(LOGGER, "Initializing MoveItReachDisplay!");
  if (!reach::plugins::DisplayBase::initialize(name, node, model)) {
    return false;
  }

  std::string param_prefix("display_config.");
  std::string planning_group;

  if (!node_->get_parameter(param_prefix + "planning_group", planning_group) ||
      !node_->get_parameter(param_prefix + "collision_mesh_package",
                            collision_mesh_package_) ||
      !node_->get_parameter(param_prefix + "collision_mesh_filename_path",
                            collision_mesh_filename_path_) ||
      !node_->get_parameter(param_prefix + "fixed_frame", fixed_frame_) ||
      !node_->get_parameter(param_prefix + "collision_mesh_frame",
                            collision_mesh_frame_) ||
      !node_->get_parameter(param_prefix + "marker_scale", marker_scale_)) {
    RCLCPP_ERROR(LOGGER,
                 "MoveIt IK Solver Plugin is missing one or more configuration "
                 "parameters");
    return false;
  }

  //    model_ = moveit::planning_interface::getSharedRobotModelLoader(node,
  //    "robot_description")->getModel();
  model_ = model;

  if (!model_) {
    RCLCPP_ERROR(LOGGER, "Failed to initialize robot model pointer");
    return false;
  }

  jmg_ = model_->getJointModelGroup(planning_group);
  if (!jmg_) {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to get joint model group for '"
                                    << planning_group << "'");
    return false;
  }

  scene_ = std::make_shared<planning_scene::PlanningScene>(model_);

  // Check that the input collision mesh frame exists
  if (!scene_->knowsFrameTransform(collision_mesh_frame_)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Specified collision mesh frame '"
                                    << collision_mesh_frame_
                                    << "' does not exist");
    return false;
  }

  // Add the collision object to the planning scene
  const std::string object_name = "reach_object";
  moveit_msgs::msg::CollisionObject obj = utils::createCollisionObject(
      collision_mesh_package_, collision_mesh_frame_, object_name);

  if (!scene_->processCollisionObjectMsg(obj)) {
    RCLCPP_ERROR(LOGGER, "Failed to add collision mesh to planning scene");
    return false;
  } else {
    RCLCPP_INFO(LOGGER, "Successfully processed collision object '%s'",
                object_name.c_str());
  }

  scene_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>(
      PLANNING_SCENE_TOPIC, 1);

  traj_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      TRAJECTORY_SCENE_TOPIC, 1);

  RCLCPP_INFO_STREAM(LOGGER,
                     "Successfully initialized MoveItReachDisplay plugin");
  return true;
}

void MoveItReachDisplay::showEnvironment() {
  moveit_msgs::msg::PlanningScene scene_msg;
  scene_->getPlanningSceneMsg(scene_msg);
  scene_pub_->publish(scene_msg);
}

void MoveItReachDisplay::updateRobotPose(
    const std::map<std::string, double>& pose) {
  std::vector<std::string> joint_names = jmg_->getActiveJointModelNames();

  std::vector<double> joints;
  if (utils::transcribeInputMap(pose, joint_names, joints)) {
    moveit_msgs::msg::PlanningScene scene_msg;
    scene_msg.is_diff = true;
    scene_msg.robot_state.is_diff = true;
    scene_msg.robot_state.joint_state.name = joint_names;
    scene_msg.robot_state.joint_state.position = joints;
    scene_pub_->publish(scene_msg);
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to transcribe input joints");
  }
}
void MoveItReachDisplay::updateRobotTrajectory(
    const std::vector<std::map<std::string, double>>& traj) {
  const size_t& size = traj.size();

  if (size > 0) {
    std::vector<std::string> joint_names = jmg_->getActiveJointModelNames();
    moveit_msgs::msg::DisplayTrajectory trajectory_msg;
    trajectory_msg.trajectory.resize(1);
    trajectory_msg.trajectory[0].joint_trajectory.joint_names = joint_names;
    trajectory_msg.trajectory[0].joint_trajectory.points.resize(size);

    for (size_t i = 0; i < traj.size(); ++i) {
      std::vector<double> joints;

      if (utils::transcribeInputMap(traj[i], joint_names, joints)) {
        if (i == 0) {
          trajectory_msg.trajectory_start.joint_state.name = joint_names;
          trajectory_msg.trajectory_start.joint_state.position = joints;
          trajectory_msg.trajectory_start.is_diff = true;
        }
        trajectory_msg.trajectory[0].joint_trajectory.points[i].positions =
            joints;
        trajectory_msg.trajectory[0].joint_trajectory.joint_names = joint_names;

      } else {
        RCLCPP_ERROR(LOGGER, "Failed to transcribe input joints");
      }
    }
    // publish trajectory
    traj_pub_->publish(trajectory_msg);
  } else {
    traj_pub_->publish(moveit_msgs::msg::DisplayTrajectory());
  }
}

void MoveItReachDisplay::showEnvironment(const std::vector<std::string>& names,
                                         const std::vector<double>& positions) {

}

}  // namespace display
}  // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::display::MoveItReachDisplay,
                       reach::plugins::DisplayBase)
