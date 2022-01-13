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
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene/planning_scene.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

const static std::string PLANNING_SCENE_TOPIC = "planning_scene_display";

namespace moveit_reach_plugins
{
namespace display
{

MoveItReachDisplay::MoveItReachDisplay()
  : reach::plugins::DisplayBase()
{

}

bool MoveItReachDisplay::initialize(std::string& name, rclcpp::Node::SharedPtr node)
{
    RCLCPP_INFO(LOGGER, "Initializing MoveItReachDisplay!");
    reach::plugins::DisplayBase::initialize(name, node);

    n_ = node;

    std::string param_prefix("display_config.");
    std::string planning_group;

    if(!node_->get_parameter(param_prefix + "planning_group", planning_group) ||
     !node_->get_parameter(param_prefix + "collision_mesh_package", collision_mesh_package_) ||
     !node_->get_parameter(param_prefix + "collision_mesh_filename_path", collision_mesh_filename_path_) ||
     !node_->get_parameter(param_prefix + "fixed_frame", fixed_frame_) ||
     !node_->get_parameter(param_prefix + "collision_mesh_frame", collision_mesh_frame_) ||
     !node_->get_parameter(param_prefix + "marker_scale", marker_scale_))
  {
    RCLCPP_ERROR(LOGGER, "MoveIt IK Solver Plugin is missing one or more configuration parameters");
    return false;
  }

    model_ = moveit::planning_interface::getSharedRobotModelLoader(node, "robot_description")->getModel();

    if(!model_)
  {
    RCLCPP_ERROR(LOGGER, "Failed to initialize robot model pointer");
    return false;
  }

  jmg_ = model_->getJointModelGroup(planning_group);
  if(!jmg_)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to get joint model group for '" << planning_group << "'");
    return false;
  }

  scene_.reset(new planning_scene::PlanningScene (model_));

  // Check that the input collision mesh frame exists
  if(!scene_->knowsFrameTransform(collision_mesh_frame_))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Specified collision mesh frame '" << collision_mesh_frame_ << "' does not exist");
    return false;
  }

  // Add the collision object to the planning scene
  const std::string object_name = "reach_object";
  moveit_msgs::msg::CollisionObject obj = utils::createCollisionObject(collision_mesh_package_, collision_mesh_frame_, object_name);

  if(!scene_->processCollisionObjectMsg(obj))
  {
    RCLCPP_ERROR(LOGGER, "Failed to add collision mesh to planning scene");
    return false;
  }else {
      RCLCPP_INFO(LOGGER, "Successfully processed collision object '%s'", object_name.c_str());
  }

  scene_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>(PLANNING_SCENE_TOPIC, 1);

  RCLCPP_INFO_STREAM(LOGGER, "Successfully initialized MoveItReachDisplay plugin");
  return true;
}

void MoveItReachDisplay::showEnvironment()
{
  while (scene_pub_->get_subscription_count() < 1)
    {
        RCLCPP_INFO(LOGGER, "No subscribers. Not showing environment...");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  moveit_msgs::msg::PlanningScene scene_msg;
  scene_->getPlanningSceneMsg(scene_msg);
  scene_pub_->publish(scene_msg);
}

void MoveItReachDisplay::updateRobotPose(const std::map<std::string, double>& pose)
{
  std::vector<std::string> joint_names = jmg_->getActiveJointModelNames();
  std::vector<double> joints;
  if(utils::transcribeInputMap(pose, joint_names, joints))
  {
    moveit_msgs::msg::PlanningScene scene_msg;
    scene_msg.is_diff = true;
    scene_msg.robot_state.is_diff = true;
    scene_msg.robot_state.joint_state.name = joint_names;
    scene_msg.robot_state.joint_state.position = joints;
    scene_pub_->publish(scene_msg);
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Failed to transcribe input joints");
  }
}

    void
    MoveItReachDisplay::showEnvironment(const std::vector<std::string> &names, const std::vector<double> &positions) {

    }

} // namespace display
} // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::display::MoveItReachDisplay, reach::plugins::DisplayBase)
