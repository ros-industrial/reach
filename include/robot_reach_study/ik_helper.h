#ifndef IK_HELPER_H
#define IK_HELPER_H

#include <robot_reach_study/reach_database.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <vector>
#include <boost/optional.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace robot_reach_study
{

//struct solution_properties
//{
//  double score;
//  double manipulability;
//  double joint_penalty;
//  double collision_distance;
//};

class IkHelper
{
public:
  IkHelper(const std::string robot_description,
           const std::string kin_group_name,
           const std::string manip_group_name);

  boost::optional<double> solveIKFromSeed(const geometry_msgs::Pose& tgt_pose,
                                          moveit::core::RobotState& seed_state,
                                          moveit::core::RobotState& goal_state);

  std::vector<std::string> reachNeighborsDirect(std::shared_ptr<robot_reach_study::Database>& db,
                                                const robot_reach_study::ReachRecord& rec);

  void reachNeighborsRecursive(std::shared_ptr<robot_reach_study::Database>& db,
                               const std::string& current_msg_id,
                               const geometry_msgs::Pose& current_pose,
                               const moveit::core::RobotState& current_state,
                               std::vector<std::string>& reached_pts);

  bool addCollisionObjectToScene(const std::string& mesh_filename,
                                 const std::string& parent_link,
                                 const std::vector<std::string>& touch_links = std::vector<std::string>());

  const moveit::core::RobotState getCurrentRobotState() const {return scene_ptr_->getCurrentState();}

  planning_scene::PlanningScenePtr getPlanningScene() const {return scene_ptr_;}

  void setSolutionAttempts(unsigned int n) {sol_attempts_ = n;}

  void setSolutionTimeout(float n) {sol_timeout_ = n;}

  void setNeighborRadius(float r) {neighbor_radius_ = r;}

  void setKinematicJointModelGroup(std::string& name) {kin_jmgroup_ = model_->getJointModelGroup(name);
                                                       kin_joint_limits_ = this->getJointLimits(kin_jmgroup_);}

  void setManipulabilityJointModelGroup(std::string& name) {manip_jmgroup_ = model_->getJointModelGroup(name);
                                                            manip_joint_limits_ = this->getJointLimits(manip_jmgroup_);}


private:
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  robot_model_loader::RobotModelLoaderPtr model_loader_;
  robot_model::RobotModelConstPtr model_;
  planning_scene::PlanningScenePtr scene_ptr_;
  ros::Publisher planning_scene_publisher_;
  const moveit::core::JointModelGroup* kin_jmgroup_;
  const moveit::core::JointModelGroup* manip_jmgroup_;
  robot_state::GroupStateValidityCallbackFn constraint_;
  unsigned int sol_attempts_;
  float sol_timeout_;
  std::vector<std::vector<double>> kin_joint_limits_;
  std::vector<std::vector<double>> manip_joint_limits_;
  float neighbor_radius_;

  bool isIKSolutionValid(moveit::core::RobotState* state,
                         const moveit::core::JointModelGroup* jmg,
                         const double* ik_solution) const;

  double getManipulability(const moveit::core::RobotState& state,
                           const moveit::core::JointModelGroup* jmg);

  double getJointPenalty(const moveit::core::RobotState& state,
                         const moveit::core::JointModelGroup* jmg);

  std::vector<std::vector<double>> getJointLimits(const moveit::core::JointModelGroup* jmg);

};

}

#endif // IK_HELPER_H
