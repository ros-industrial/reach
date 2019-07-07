#ifndef MOVEIT_REACH_PLUGINS_EVALUATION_MANIPULABILITY_EVALUATION_H
#define MOVEIT_REACH_PLUGINS_EVALUATION_MANIPULABILITY_EVALUATION_H

#include <reach/plugins/evaluation_base.h>

namespace moveit
{
namespace core
{
class RobotModel;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;
class JointModelGroup;
}
}

namespace moveit_reach_plugins
{
namespace evaluation
{

class ManipulabilityMoveIt : public reach::plugins::EvaluationBase
{
public:

  ManipulabilityMoveIt();

  virtual bool initialize(XmlRpc::XmlRpcValue& config) override;

  virtual double calculateScore(const std::map<std::string, double>& pose) override;

private:

  moveit::core::RobotModelConstPtr model_;

  const moveit::core::JointModelGroup* jmg_;
};

} // namespace evaluation
} // namepsace moveit_reach_plugins

#endif // MOVEIT_REACH_PLUGINS_EVALUATION_MANIPULABILITY_EVALUATION_H
