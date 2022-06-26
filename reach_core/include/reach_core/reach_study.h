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

#include <reach_core/ik_helper.h>
#include <reach_core/plugins/ik_solver_base.h>
#include <reach_core/plugins/target_pose_generator_base.h>

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
  struct Parameters
  {
    int max_steps;
    float step_improvement_threshold;
    float radius;
  };

  ReachStudy(plugins::IKSolverBase::ConstPtr ik_solver, plugins::TargetPoseGeneratorBase::ConstPtr pose_generator,
             const Parameters params, const std::string& study_name);

  void load(const std::string& filename);
  void run();
  void optimize();
  void save(const std::string& filename) const;

  StudyResults getResults() const;
  ReachDatabase::ConstPtr getDatabase() const;

  std::tuple<double, double> getAverageNeighborsCount() const;

private:
  Parameters params_;
  ReachDatabase::Ptr db_;

  // Plugins
  plugins::IKSolverBase::ConstPtr ik_solver_;
  const plugins::VectorIsometry3d target_poses_;

  SearchTreePtr search_tree_ = nullptr;
};

}  // namespace core
}  // namespace reach

#endif  // REACH_CORE_REACH_STUDY_H
