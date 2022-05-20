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
#ifndef REACH_CORE_PARAMETERS_H
#define REACH_CORE_PARAMETERS_H

#include <string>
#include <vector>
//#include <xmlrpcpp/XmlRpcValue.h>

namespace reach {
namespace core {

/**
 * @brief The StudyResults struct
 */
struct StudyResults {
  float total_pose_score = 0.0f;
  float norm_total_pose_score = 0.0f;
  float reach_percentage = 0.0f;
  float avg_num_neighbors = 0.0f;
  float avg_joint_distance = 0.0f;
};

struct StudyOptimization {
  int max_steps;
  float step_improvement_threshold;
  float radius;
};

/**
 * @brief The StudyParameters struct contains all necessary parameters for the
 * reach study
 */
struct StudyParameters {
  //  XmlRpc::XmlRpcValue ik_solver_config;
  //  XmlRpc::XmlRpcValue display_config;
  std::string ik_solver_config_name;
  std::string display_config_name;
  StudyOptimization optimization;
  std::string config_name;
  std::string results_package;
  std::string results_directory;
  std::string pcd_package;
  std::string pcd_filename_path;
  bool visualize_results;
  bool get_neighbors;
  std::vector<std::string> compare_dbs;
  std::vector<std::string> visualize_dbs;
  std::string fixed_frame;
  std::string object_frame;
  std::string planning_group;
  bool run_initial_study_only;
  std::vector<double> initial_seed_state;
  bool keep_running;
};

}  // namespace core
}  // namespace reach

#endif  // REACH_CORE_PARAMETERS_H
