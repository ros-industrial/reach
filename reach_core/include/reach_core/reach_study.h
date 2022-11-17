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

#include <reach_core/interfaces/display.h>
#include <reach_core/interfaces/evaluator.h>
#include <reach_core/interfaces/ik_solver.h>
#include <reach_core/interfaces/target_pose_generator.h>
#include <reach_core/interfaces/logger.h>
#include <reach_core/utils.h>

#include <boost/filesystem/path.hpp>

namespace reach
{
/**
 * @brief Class for performing reachability analysis
 */
class ReachStudy
{
public:
  struct Parameters
  {
    int max_steps;
    float step_improvement_threshold;
    float radius;
  };

  ReachStudy(IKSolver::ConstPtr ik_solver, Evaluator::ConstPtr evaluator, TargetPoseGenerator::ConstPtr pose_generator,
             Display::ConstPtr display, Logger::ConstPtr logger, const Parameters params,
             const std::string& study_name);

  /** @brief Loads the results of a reach study from file */
  void load(const std::string& filename);

  /** @brief Runs the first iteration of the reach study process */
  void run();

  /** @brief Runs successive iterations of the reach study process */
  void optimize();

  /** @brief Saves the results of the reach study to a file */
  void save(const std::string& filename) const;

  StudyResults getResults() const;
  ReachDatabase::ConstPtr getDatabase() const;

  /**
   * @brief Finds the average number of neighbors that are "recursively" reachable from each target pose and the average
   * joint distance of the set of neighbors from each target pose
   *
   * @details Finding the "recursive" reachability from a given target pose means:
   *   1. Identifying the neighbors of the target pose within a given radius (defined by a reach study parameter)
   *   2. For each neighbor, attempt to solve IK for that neighbor using the IK solution of the original pose as the IK
   * seed
   *   3. If the neighbor is reachable, repeat this process using the neighbor as the source pose. Continue until a
   * source pose has no more reachable neighbors
   *
   * The result of this function can be a proxy for the motion planning ability of a robot. The larger the average
   * number of neighbors and greater the average joint distance to those neighbors, the further the robot will be able
   * to travel away from a given starting location while maintaining a consistent IK configuration.
   *
   * @return tuple, where the first element is the average number of neighbors that is "recursively" reachable from each
   * target pose, and the second element is the average joint distance between the set of neighbors and the target pose
   */
  std::tuple<double, double> getAverageNeighborsCount() const;

private:
  Parameters params_;
  ReachDatabase::Ptr db_;

  // Plugins
  IKSolver::ConstPtr ik_solver_;
  Evaluator::ConstPtr evaluator_;
  Display::ConstPtr display_;
  Logger::ConstPtr logger_;

  const VectorIsometry3d target_poses_;

  SearchTreePtr search_tree_ = nullptr;
};

/**
 * @brief Helper function for configuring a ReachStudy object and performing a reach study
 */
void runReachStudy(const YAML::Node& config, const std::string& config_name = "reach_study",
                   const boost::filesystem::path& results_dir = "/tmp", const bool wait_after_completion = true);

}  // namespace reach

#endif  // REACH_CORE_REACH_STUDY_H
