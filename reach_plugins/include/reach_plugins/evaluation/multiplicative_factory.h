#ifndef REACH_PLUGINS_EVALUATION_MULTIPLICATIVE_FACTORY_H
#define REACH_PLUGINS_EVALUATION_MULTIPLICATIVE_FACTORY_H

#include "evaluation_base.h"

namespace reach_plugins
{
namespace evaluation
{

class MultiplicativeFactory : public EvaluationBase
{
public:

  MultiplicativeFactory();

  virtual bool initialize(XmlRpc::XmlRpcValue& config) override;

  virtual double calculateScore(const std::map<std::string, double>& pose) override;

private:

  std::vector<EvaluationBasePtr> eval_plugins_;
};

} // namespace evaluation
} // namespace reach_plugins

#endif // REACH_PLUGINS_EVALUATION_MULTIPLICATIVE_FACTORY_H
