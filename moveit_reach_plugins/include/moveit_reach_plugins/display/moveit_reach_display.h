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
#ifndef MOVEIT_REACH_PLUGINS_MOVEIT_REACH_DISPLAY_H
#define MOVEIT_REACH_PLUGINS_MOVEIT_REACH_DISPLAY_H

#include <rclcpp/rclcpp.hpp>

#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <reach_core/plugins/reach_display_base.h>

namespace planning_scene {
class PlanningScene;
typedef std::shared_ptr<PlanningScene> PlanningScenePtr;
}  // namespace planning_scene

namespace moveit_reach_plugins {
namespace {
const rclcpp::Logger LOGGER =
    rclcpp::get_logger("moveit_reach_plugins.MoveItReachDisplay");
}
namespace display {

class MoveItReachDisplay : public reach::plugins::DisplayBase {
 public:
  MoveItReachDisplay();
  ~MoveItReachDisplay() {
    model_.reset();
    scene_.reset();
    //    delete jmg_;
    scene_pub_.reset();
    traj_pub_.reset();
  }

  bool initialize(
      std::string& name, rclcpp::Node::SharedPtr node,
      const std::shared_ptr<const moveit::core::RobotModel> model) override;

  virtual void showEnvironment() override;

  virtual void showEnvironment(const std::vector<std::string>& names,
                               const std::vector<double>& positions) override;

  virtual void updateRobotPose(
      const std::map<std::string, double>& pose) override;

  virtual void updateRobotTrajectory(
      const std::vector<std::map<std::string, double>>& traj) override;

 private:
  moveit::core::RobotModelConstPtr model_;

  planning_scene::PlanningScenePtr scene_;

  const moveit::core::JointModelGroup* jmg_;

  std::string collision_mesh_package_;
  std::string collision_mesh_filename_path_;
  std::string collision_mesh_frame_;

  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr traj_pub_;
};

}  // namespace display
}  // namespace moveit_reach_plugins

#endif  // MOVEIT_REACH_PLUGINS_MOVEIT_REACH_DISPLAY_H
