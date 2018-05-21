#ifndef REACH_PLUGINS_MOVEIT_REACH_DISPLAY_H
#define REACH_PLUGINS_MOVEIT_REACH_DISPLAY_H

#include "reach_display_base.h"

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
namespace display
{

class MoveItReachDisplay : public ReachDisplayBase
{
public:

  MoveItReachDisplay();

  virtual bool initialize(XmlRpc::XmlRpcValue& config) override;

  virtual void showEnvironment() override;

  virtual void updateRobotPose(const std::vector<double>& pose) override;

private:

  moveit::core::RobotModelConstPtr model_;

  planning_scene::PlanningScenePtr scene_;

  const moveit::core::JointModelGroup* jmg_;

  std::string collision_mesh_filename_;

  std::string collision_mesh_frame_;

  ros::NodeHandle nh_;

  ros::Publisher scene_pub_;
};

} // namespace display
} // namespace reach_plugins

#endif // REACH_PLUGINS_MOVEIT_REACH_DISPLAY_H
