#ifndef REACH_CORE_PARAMETERS_H
#define REACH_CORE_PARAMETERS_H

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
struct StudyResults
{
  float total_pose_score = 0.0f;
  float norm_total_pose_score = 0.0f;
  float reach_percentage = 0.0f;
  float avg_num_neighbors = 0.0f;
  float avg_joint_distance = 0.0f;
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
  StudyOptimization optimization;
  std::string config_name;
  std::string results_directory;
  std::string pcd_filename;
  bool visualize_results;
  bool get_neighbors;
  std::vector<std::string> compare_dbs;
  std::string fixed_frame;
  std::string object_frame;
};

} // namespace core
} // namespace reach

#endif // REACH_CORE_PARAMETERS_H
