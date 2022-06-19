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

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <string>
#include <vector>
#include <xmlrpcpp/XmlRpcValue.h>

namespace reach
{
namespace core
{
/**
 * @brief The StudyResults struct
 */
class StudyResults
{
public:
  float total_pose_score = 0.0f;
  float norm_total_pose_score = 0.0f;
  float reach_percentage = 0.0f;
  float avg_num_neighbors = 0.0f;
  float avg_joint_distance = 0.0f;

private:
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned /*version*/)
  {
    ar & BOOST_SERIALIZATION_NVP(total_pose_score);
    ar & BOOST_SERIALIZATION_NVP(norm_total_pose_score);
    ar & BOOST_SERIALIZATION_NVP(reach_percentage);
    ar & BOOST_SERIALIZATION_NVP(avg_num_neighbors);
    ar & BOOST_SERIALIZATION_NVP(avg_joint_distance);
  }
};

struct StudyOptimization
{
  int max_steps;
  float step_improvement_threshold;
  float radius;
};

/**
 * @brief The StudyParameters struct contains all necessary parameters for the reach study
 */
struct StudyParameters
{
  XmlRpc::XmlRpcValue ik_solver_config;
  XmlRpc::XmlRpcValue display_config;
  XmlRpc::XmlRpcValue target_pose_generator_config;
  StudyOptimization optimization;
  std::string config_name;
  std::string results_directory;
  bool visualize_results;
  bool get_neighbors;
  std::vector<std::string> compare_dbs;
};

}  // namespace core
}  // namespace reach

#endif  // REACH_CORE_PARAMETERS_H
