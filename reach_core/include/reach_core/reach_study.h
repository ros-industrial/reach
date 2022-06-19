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
#include <reach_core/plugins/target_pose_generator_base.h>

#include <pluginlib/class_loader.h>

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
  ReachStudy();

  /**
   * @brief run
   * @param sp
   * @return
   */
  void run(const StudyParameters& sp);

private:
  void initializeStudy();

  void runInitialReachStudy();

  void optimizeReachStudyResults();

  void getAverageNeighborsCount();

  bool compareDatabases();

  StudyParameters sp_;
  ReachDatabase::Ptr db_;

  // Plugins
  pluginlib::ClassLoader<reach::plugins::IKSolverBase> solver_loader_;
  pluginlib::ClassLoader<reach::plugins::DisplayBase> display_loader_;
  pluginlib::ClassLoader<reach::plugins::TargetPoseGeneratorBase> target_pose_generator_loader_;
  plugins::IKSolverBase::Ptr ik_solver_;
  plugins::DisplayBase::Ptr display_;
  plugins::VectorIsometry3d target_poses_;

  ReachVisualizer::Ptr visualizer_;
  SearchTreePtr search_tree_;
  std::string dir_;
  std::string results_dir_;
};

}  // namespace core
}  // namespace reach

#endif  // REACH_CORE_REACH_STUDY_H
