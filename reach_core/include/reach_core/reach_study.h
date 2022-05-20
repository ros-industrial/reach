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
#ifndef REACH_CORE_REACH_STUDY_H
#define REACH_CORE_REACH_STUDY_H

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <rclcpp/rclcpp.hpp>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_loader.hpp>
#include <reach_core/ik_helper.h>
#include <reach_core/plugins/ik_solver_base.h>
#include <reach_core/reach_visualizer.h>
#include <reach_core/study_parameters.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>

//
// namespace moveit
//{
//    namespace core
//    {
//        class RobotModel;
//        typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;
//        class JointModelGroup;
//    }
//}

namespace reach {
namespace core {

/**
 * @brief The ReachStudy class
 */
class ReachStudy {
 public:
  /**
   * @brief ReachStudy
   * @param nh
   */
  ReachStudy(const rclcpp::Node::SharedPtr node);

  ~ReachStudy();

  /**
   * @brief run
   * @param sp
   * @return
   */
  bool run(const StudyParameters &sp);

  std::shared_ptr<rclcpp::Node> get_node() {
    if (!node_.get()) {
      throw std::runtime_error("Node hasn't been initialized yet!");
    }
    return node_;
  }

 private:
  bool initializeStudy(const StudyParameters &sp);

  bool getReachObjectPointCloud();

  void runInitialReachStudy();

  void optimizeReachStudyResults();

  void getAverageNeighborsCount();

  bool compareDatabases();

  bool visualizeDatabases();

  StudyParameters sp_;

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_;

  ReachDatabasePtr db_;

  // Plugins
  pluginlib::ClassLoader<reach::plugins::IKSolverBase> solver_loader_;
  pluginlib::ClassLoader<reach::plugins::DisplayBase> display_loader_;
  reach::plugins::IKSolverBasePtr ik_solver_;
  reach::plugins::DisplayBasePtr display_;

  ReachVisualizerPtr visualizer_;

  SearchTreePtr search_tree_;

  std::string dir_;

  std::string results_dir_;

  sensor_msgs::msg::PointCloud2 cloud_msg_;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ps_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr done_pub_;

  // robot model
  moveit::core::RobotModelConstPtr model_;
};

}  // namespace core
}  // namespace reach

#endif  // REACH_CORE_REACH_STUDY_H
