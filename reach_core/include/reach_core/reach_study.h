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

#include <reach_core/study_parameters.h>
#include <reach_core/ik_helper.h>
#include <reach_core/reach_visualizer.h>
#include <reach_core/plugins/ik_solver_base.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/point_cloud.hpp>
#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace reach
{
  namespace core
  {

    /**
 * @brief The ReachStudy class
 */
    class ReachStudy
    {
    public:
      /**
   * @brief ReachStudy
   * @param nh
   */
      ReachStudy(const ros::NodeHandle &nh);

      /**
   * @brief run
   * @param sp
   * @return
   */
      bool run(const StudyParameters &sp);

    private:
      bool initializeStudy();

      bool getReachObjectPointCloud();

      void runInitialReachStudy();

      void optimizeReachStudyResults();

      void getAverageNeighborsCount();

      bool compareDatabases();

      ros::NodeHandle nh_;

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

      sensor_msgs::PointCloud2 cloud_msg_;
    };

  } // namespace core
} // namespace reach

#endif // REACH_CORE_REACH_STUDY_H
