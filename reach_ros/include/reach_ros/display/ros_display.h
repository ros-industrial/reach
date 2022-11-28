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
#ifndef REACH_ROS_ROS_REACH_DISPLAY_H
#define REACH_ROS_ROS_REACH_DISPLAY_H

#include <reach/interfaces/display.h>

#include <interactive_markers/interactive_marker_server.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

namespace reach_ros
{
namespace display
{
class ROSDisplay : public reach::Display
{
public:
  ROSDisplay(std::string kinematic_base_frame, double marker_scale);

  void showEnvironment() const override;
  void updateRobotPose(const std::map<std::string, double>& pose) const override;
  void showResults(const reach::ReachResult& db) const override;
  void showReachNeighborhood(const std::map<std::size_t, reach::ReachRecord>& neighborhood) const override;

  void setCollisionMarker(std::string collision_mesh_filename, const std::string collision_mesh_frame);

protected:
  const std::string kinematic_base_frame_;
  const double marker_scale_;
  visualization_msgs::Marker collision_marker_;

  // ROS comoponents
  ros::NodeHandle nh_;
  ros::Publisher joint_state_pub_;
  ros::Publisher mesh_pub_;
  ros::Publisher neighbors_pub_;
  mutable interactive_markers::InteractiveMarkerServer server_;
};

struct ROSDisplayFactory : public reach::DisplayFactory
{
  reach::Display::ConstPtr create(const YAML::Node& config) const override;
};

}  // namespace display
}  // namespace reach_ros

#endif  // REACH_ROS_ROS_REACH_DISPLAY_H
