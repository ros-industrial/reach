#ifndef REACH_PLUGINS_EVALUATION_JOINT_PENALTY_MOVEIT_H
#define REACH_PLUGINS_EVALUATION_JOINT_PENALTY_MOVEIT_H

#include "evaluation_base.h"

namespace moveit
{
namespace core
{
class RobotModel;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;
class JointModelGroup;
}
}

namespace reach_plugins
{
namespace evaluation
{

class JointPenaltyMoveIt : public EvaluationBase
{
public:

  JointPenaltyMoveIt();

  virtual bool initialize(XmlRpc::XmlRpcValue& config) override;

  virtual double calculateScore(const std::vector<double>& pose) override;

private:

  std::vector<std::vector<double>> getJointLimits();

  moveit::core::RobotModelConstPtr model_;

  const moveit::core::JointModelGroup* jmg_;

  std::vector<std::vector<double>> joint_limits_;
};

} // namespace evaluation
} // namespace reach_plugins

#endif // REACH_PLUGINS_EVALUATION_JOINT_PENALTY_MOVEIT_H
