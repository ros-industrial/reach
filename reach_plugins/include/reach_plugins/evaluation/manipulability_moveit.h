#ifndef REACH_PLUGINS_EVALUATION_MANIPULABILITY_EVALUATION_H
#define REACH_PLUGINS_EVALUATION_MANIPULABILITY_EVALUATION_H

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

class ManipulabilityMoveIt : public EvaluationBase
{
public:

  ManipulabilityMoveIt();

  virtual bool initialize(XmlRpc::XmlRpcValue& config) override;

  virtual double calculateScore(const std::vector<double>& pose) override;

private:

  moveit::core::RobotModelConstPtr model_;

  const moveit::core::JointModelGroup* jmg_;
};

} // namespace evaluation
} // reach_plugins

#endif // REACH_PLUGINS_EVALUATION_MANIPULABILITY_EVALUATION_H
