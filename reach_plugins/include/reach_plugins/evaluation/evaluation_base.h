#ifndef REACH_PLUGINS_EVALUATION_EVALUATION_BASE
#define REACH_PLUGINS_EVALUATION_EVALUATION_BASE

#include <boost/shared_ptr.hpp>
#include <vector>
#include <xmlrpcpp/XmlRpcValue.h>

namespace reach_plugins
{
namespace evaluation
{

/**
 * @brief The EvaluationBase class
 */
class EvaluationBase
{
public:

  EvaluationBase()
  {

  }

  virtual ~EvaluationBase()
  {

  }

  /**
   * @brief initialize
   * @param config
   */
  virtual bool initialize(XmlRpc::XmlRpcValue& config) = 0;

  /**
   * @brief calculateScore
   * @param pose
   * @return
   */
  virtual double calculateScore(const std::map<std::string, double>& pose) = 0;

};
typedef boost::shared_ptr<EvaluationBase> EvaluationBasePtr;

} // namespace evaluation
} // namespace reach_plugins

#endif // REACH_PLUGINS_EVALUATION_EVALUATION_BASE
