#ifndef REACH_CORE_PARAMETERS_H
#define REACH_CORE_PARAMETERS_H

#include <string>
#include <vector>

namespace reach
{
namespace core
{

/**
 * @brief The StudyParameters struct contains all necessary parameters for the reach study
 */
struct StudyParameters
{
  std::string config_name;              /** @brief reach study configuration name **/
  std::string fixed_frame;              /** @brief root, fixed frame of the URDF **/
  std::string results_directory;        /** @brief directory in which the study results will be saved **/
  std::string object_frame;             /** @brief frame of the reach object in the URDF **/
  std::string mesh_filename;            /** @brief filename which contains the reach object mesh **/
  std::string pcd_filename;             /** @brief filename which contains the point cloud representation of the reach object **/
  float optimization_radius;            /** @brief The radius around a given point which identifies neighboring points used to optimize the reach study results **/
  std::string kin_group_name;           /** @brief planning group used to solve for IK **/
  std::string manip_group_name;         /** @brief planning group used to calculate pose scoring **/
  bool get_neighbors;                   /** @brief flag for evaluating robot work area **/
  bool visualize_results;               /** @brief flag for publishing reach study data/markers for Rviz **/
  std::vector<std::string> compare_dbs; /** @brief list of database names with which to compare to the current database **/
  int cost_function;                    /** @brief enumeration defining the method used to score robot poses **/
  float distance_threshold;             /** @brief minimum distance from collision that the robot must be for the IK solution to be considered valid **/
};

struct StudySetup
{
  std::string results_directory;        /** @brief directory in which the study results will be saved **/
  std::string fixed_frame;              /** @brief root, fixed frame of the URDF **/
  std::string object_frame;             /** @brief frame of the reach object in the URDF **/
};

struct StudyConfig
{
  std::string config_name;              /** @brief reach study configuration name **/
  std::string pcd_filename;             /** @brief filename which contains the point cloud representation of the reach object **/
  std::string mesh_filename;            /** @brief filename which contains the reach object mesh **/
  std::string kin_group_name;           /** @brief planning group used to solve for IK **/
  std::string manip_group_name;         /** @brief planning group used to calculate pose scoring **/
};

struct StudyCalculation
{
  float optimization_radius;            /** @brief The radius around a given point which identifies neighboring points used to optimize the reach study results **/
  bool get_neighbors;                   /** @brief flag for evaluating robot work area **/
  bool visualize_results;               /** @brief flag for publishing reach study data/markers for Rviz **/
  int cost_function;                    /** @brief enumeration defining the method used to score robot poses **/
  float distance_threshold;             /** @brief minimum distance from collision that the robot must be for the IK solution to be considered valid **/
};

struct StudyParameters2
{
  StudySetup setup;
  StudyConfig config;
  StudyCalculation calc;
};

/**
 * @brief The CostFunction enumeration defines which quantities are used to optimize the results of the reach study
 */
enum CostFunction
{
  M = 0,          // Manipulability
  M_JP = 1,       // Manipulability and Joint Penalty
  M_JP_DP = 2,    // Manipulability, Joint Penalty, and Distance Penalty
  M_DP = 3,       // Manipulability and Distance Penalty
  M_DP_DT = 4,    // Manipulability, Distance Penalty, and Distance Threshold
  M_JP_DP_DT = 5  // Manipulabilty, Joint Penalty, Distance Penalty, and Distance Threshold
};

/**
 * @brief The IKParameters struct
 */
struct IKParameters
{
  int sol_attempts = 5;
  float sol_timeout = 0.05;
  float neighbor_radius = 1.0;
  float dist_threshold = 0.0;
};

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

}
}

#endif // REACH_CORE_PARAMETERS_H
