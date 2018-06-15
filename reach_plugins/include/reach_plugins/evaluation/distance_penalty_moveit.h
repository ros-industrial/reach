#ifndef REACH_PLUGINS_EVALUATION_DISTANCE_PENALTY_MOVEIT_H
#define REACH_PLUGINS_EVALUATION_DISTANCE_PENALTY_MOVEIT_H

#include "evaluation_base.h"
#include <moveit_msgs/PlanningScene.h>

namespace moveit
{
namespace core
{
class RobotModel;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;
class JointModelGroup;
}
}

namespace planning_scene
{
class PlanningScene;
typedef std::shared_ptr<PlanningScene> PlanningScenePtr;
}

namespace reach_plugins
{
namespace evaluation
{

class DistancePenaltyMoveIt : public EvaluationBase
{
public:

  DistancePenaltyMoveIt();

  virtual bool initialize(XmlRpc::XmlRpcValue& config) override;

  virtual double calculateScore(const std::map<std::string, double>& pose) override;

private:

  moveit::core::RobotModelConstPtr model_;

  const moveit::core::JointModelGroup* jmg_;

  planning_scene::PlanningScenePtr scene_;

  double dist_threshold_;

  int exponent_;

  std::string collision_mesh_filename_;

  std::string collision_mesh_frame_;

  std::vector<std::string> touch_links_;
};

} // namespace evaluation
} // namespace reach_plugins

#endif // REACH_PLUGINS_EVALUATION_DISTANCE_PENALTY_MOVEIT_H
