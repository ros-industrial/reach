#ifndef REACH_CORE_PLUGINS_MULTIPLICATIVE_FACTORY_H
#define REACH_CORE_PLUGINS_MULTIPLICATIVE_FACTORY_H

#include "reach/plugins/evaluation_base.h"
#include "pluginlib/class_loader.h"

namespace reach
{
namespace plugins
{

class MultiplicativeFactory : public EvaluationBase
{
public:

  MultiplicativeFactory();

  virtual bool initialize(XmlRpc::XmlRpcValue& config) override;

  virtual double calculateScore(const std::map<std::string, double>& pose) override;

private:

  std::vector<EvaluationBasePtr> eval_plugins_;  

  pluginlib::ClassLoader<EvaluationBase> class_loader_;
};

} // namespace plugins
} // namespace reach

#endif // REACH_CORE_PLUGINS_MULTIPLICATIVE_FACTORY_H
