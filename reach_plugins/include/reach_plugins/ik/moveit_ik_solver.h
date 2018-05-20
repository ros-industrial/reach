#ifndef REACH_PLUGINS_IK_MOVEIT_IK_SOLVER_H
#define REACH_PLUGINS_IK_MOVEIT_IK_SOLVER_H

#include "ik_solver_base.h"
#include <reach_plugins/evaluation/evaluation_base.h>
#include <moveit_msgs/PlanningScene.h>

namespace moveit
{
namespace core
{
class RobotModel;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;
class JointModelGroup;
class RobotState;
}
}

namespace planning_scene
{
class PlanningScene;
typedef std::shared_ptr<PlanningScene> PlanningScenePtr;
}

namespace ros
{
class NodeHandle;
class Subscriber;
}

namespace reach_plugins
{
namespace ik
{

class MoveItIKSolver : public IKSolverBase
{
public:

  MoveItIKSolver();

  virtual bool initialize(XmlRpc::XmlRpcValue& config) override;

  virtual boost::optional<double> solveIKFromSeed(const Eigen::Affine3d& target,
                                                  const std::vector<double> &seed,
                                                  std::vector<double> &solution) override;

protected:

  void updatePlanningScene(const moveit_msgs::PlanningSceneConstPtr& msg);

  bool isIKSolutionValid(moveit::core::RobotState* state,
                         const moveit::core::JointModelGroup* jmg,
                         const double* ik_solution) const;

  moveit::core::RobotModelConstPtr model_;

  planning_scene::PlanningScenePtr scene_;

  const moveit::core::JointModelGroup* jmg_;

  evaluation::EvaluationBasePtr eval_;

  double distance_threshold_;

  ros::NodeHandle nh_;

  ros::Subscriber planning_scene_sub_;
};

} // namespace ik
} // namespack reach_plugins

#endif // REACH_PLUGINS_IK_MOVEIT_IK_SOLVER_H
