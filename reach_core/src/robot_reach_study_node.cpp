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
#include "reach_core/reach_study.h"
#include "reach_core/study_parameters.h"

template <typename T>
bool get(const ros::NodeHandle& nh, const std::string& key, T& val)
{
  if (!nh.getParam(key, val))
  {
    ROS_ERROR_STREAM("Failed to get '" << key << "' parameter");
    return false;
  }
  return true;
}

bool getStudyParameters(ros::NodeHandle& nh, reach::core::StudyParameters& sp)
{
  if (!get(nh, "config_name", sp.config_name) || !get(nh, "fixed_frame", sp.fixed_frame) ||
      !get(nh, "results_directory", sp.results_directory) || !get(nh, "object_frame", sp.object_frame) ||
      !get(nh, "pcd_filename", sp.pcd_filename) || !get(nh, "optimization/radius", sp.optimization.radius) ||
      !get(nh, "optimization/max_steps", sp.optimization.max_steps) ||
      !get(nh, "optimization/step_improvement_threshold", sp.optimization.step_improvement_threshold) ||
      !get(nh, "get_avg_neighbor_count", sp.get_neighbors) || !get(nh, "compare_dbs", sp.compare_dbs) ||
      !get(nh, "visualize_results", sp.visualize_results) || !get(nh, "ik_solver_config", sp.ik_solver_config) ||
      !get(nh, "display_config", sp.display_config))
  {
    return false;
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_reach_study_node");
  ros::NodeHandle pnh("~"), nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Get the study parameters
  reach::core::StudyParameters sp;
  if (!getStudyParameters(pnh, sp))
  {
    return -1;
  }

  // Initialize the reach study
  reach::core::ReachStudy rs(nh);

  // Run the reach study
  if (!rs.run(sp))
  {
    ROS_ERROR("Unable to perform the reach study");
    return -1;
  }

  ros::waitForShutdown();

  return 0;
}
