#ifndef IK_HELPER_H
#define IK_HELPER_H

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <boost/optional.hpp>
#include <robot_reach_study/reach_database.h>
#include <ros/ros.h>

namespace robot_reach_study
{

/**
 * @brief The IkHelper class solves the robot's inverse kinematics for given target poses and contains tools to optimize
 * robot poses based on neighboring poses and analyze the total work area of the robot from a given pose
 */
class IkHelper
{
public:

  /**
   * @brief Constructor for the IkHelper class
   * @param robot_description
   * @param kin_group_name name of the MoveIt planning group with which to solve IK
   * @param manip_group_name name of the MoveIt planning group with which to evaluate manipulability
   */
  IkHelper(const std::string kin_group_name,
           const std::string manip_group_name);

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
   * @brief solveIKFromSeed attempts to find a valid IK solution for the given target pose starting from the input seed state.
   * If a solution is found, the resulting Robot State is saved into the goal_state object and the pose is scored according to
   * the specified cost function.
   * @param tgt_pose
   * @param seed_state
   * @param goal_state
   * @return the score of the particular pose if valid IK solution is found, nothing if no valid IK solution is found
   */
  boost::optional<double> solveIKFromSeed(const geometry_msgs::Pose& tgt_pose,
                                          moveit::core::RobotState& seed_state,
                                          moveit::core::RobotState& goal_state);

  /**
   * @brief reachNeighborsDirect finds all of the neighboring target poses within a specified radius of the target pose of the input
   * ReachRecord. An IK solution is attempted for the identified neighboring poses; if a solution is found with a greater score than
   * that particular pose's current recorded score, the database entry is overwritten with the new pose and score. This function
   * can be thought of as a localized optimization of poses
   * @param db
   * @param rec
   * @return a vector of ReachRecord keys representing the neighboring target poses that were reachable from the input target pose
   */
  std::vector<std::string> reachNeighborsDirect(std::shared_ptr<robot_reach_study::Database>& db,
                                                const robot_reach_study::ReachRecord& rec);

  /**
   * @brief reachNeighborsRecursive finds all of the neighboring target poses within a specified radius of the target pose of the input
   * ReachRecord. An IK solution is attempted for the identified neighboring poses. If a solution is found at a neighboring pose, this
   * process is repeated, and the function is called recursively until no IK solution can be found for any of the current pose's neighbors.
   * This function effectively identifies the accessible work area of the robot from the input pose.
   * @param db
   * @param msg
   * @param reached_pts a vector of ReachRecord keys representing the neighboring target poses that were reachable from the input target pose
   * @param joint_distance the cumulative distance in joint space that was required to travel to all of the identified reachable points. This
   * number can be used to quantify if a particular robot configuration is more "efficient" at travelling between neighboring poses
   */
  void reachNeighborsRecursive(std::shared_ptr<robot_reach_study::Database>& db,
                               const robot_reach_study::ReachRecord& msg,
                               std::vector<std::string>& reached_pts,
                               double& joint_distance);

  /**
   * @brief addCollisionObjectToScene adds the reach object to the planning scene as a collision object
   * @param mesh_filename
   * @param parent_link
   * @param touch_links the links of the URDF that are allowed to come in contact with the collision object
   * @return true on success, false on failure
   */
  bool addCollisionObjectToScene(const std::string& mesh_filename,
                                 const std::string& parent_link,
                                 const std::vector<std::string>& touch_links = std::vector<std::string>());

  /**
   * @brief getCurrentRobotState
   * @return
   */
  const moveit::core::RobotState getCurrentRobotState() const {return scene_ptr_->getCurrentState();}

  /**
   * @brief getPlanningScene
   * @return
   */
  planning_scene::PlanningScenePtr getPlanningScene() const {return scene_ptr_;}

  /**
   * @brief setSolutionAttempts sets the number of attempts with which to try to find a valid IK solution
   * @param n
   */
  void setSolutionAttempts(unsigned int n) {sol_attempts_ = n;}

  /**
   * @brief setSolutionTimeout sets the timeout after which the the IK solution process will be cancelled
   * @param n
   */
  void setSolutionTimeout(float n) {sol_timeout_ = n;}

  /**
   * @brief setNeighborRadius sets the radius with which to search for neighboring points around any given point
   * @param r
   */
  void setNeighborRadius(float r) {neighbor_radius_ = r;}

  /**
   * @brief setKinematicJointModelGroup sets the joint model group which will be used to solve IK
   * @param name
   */
  void setKinematicJointModelGroup(const std::string& name) {kin_jmgroup_ = model_->getJointModelGroup(name);
                                                             kin_joint_limits_ = this->getJointLimits(kin_jmgroup_);}

  /**
   * @brief getKinematicJointModelGroupName
   * @return
   */
  const std::string getKinematicJointModelGroupName() {return kin_jmgroup_->getName();} const

  /**
   * @brief setManipulabilityJointModelGroup sets the joint model group with which the pose score will be calculated
   * @param name
   */
  void setManipulabilityJointModelGroup(const std::string& name) {manip_jmgroup_ = model_->getJointModelGroup(name);
                                                                  manip_joint_limits_ = this->getJointLimits(manip_jmgroup_);}

  /**
   * @brief getManipulabilityJointModelGroupName
   * @return
   */
  const std::string getManipulabilityJointModelGroupName() {return manip_jmgroup_->getName();} const

  /**
   * @brief setCostFunction sets the cost function which will be used to determine the score of a particular pose
   * @param cf
   */
  void setCostFunction(const CostFunction cf) {cost_function_ = cf;}

  /**
   * @brief setDistanceThreshold sets the minimum distance to collision that is allowed between the robot and the reach object
   * @param d
   */
  void setDistanceThreshold(const float d) {dist_threshold_ = d;}

private:

  robot_model_loader::RobotModelLoaderPtr model_loader_;
  robot_model::RobotModelConstPtr model_;
  planning_scene::PlanningScenePtr scene_ptr_;
  ros::Publisher planning_scene_publisher_;
  const moveit::core::JointModelGroup* kin_jmgroup_;
  const moveit::core::JointModelGroup* manip_jmgroup_;
  robot_state::GroupStateValidityCallbackFn constraint_;
  std::vector<std::vector<double>> kin_joint_limits_;
  std::vector<std::vector<double>> manip_joint_limits_;

  CostFunction cost_function_ = IkHelper::CostFunction::M;
  int sol_attempts_ = 5;
  float sol_timeout_ = 0.05;
  float neighbor_radius_ = 1.0;
  float dist_threshold_ = 0.0;

  bool isIKSolutionValid(moveit::core::RobotState* state,
                         const moveit::core::JointModelGroup* jmg,
                         const double* ik_solution) const;

  std::vector<std::vector<double>> getJointLimits(const moveit::core::JointModelGroup* jmg);

};

}

#endif // IK_HELPER_H
