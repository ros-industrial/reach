#ifndef REACH_PLUGINS_IK_IK_SOLVER_BASE_H
#define REACH_PLUGINS_IK_IK_SOLVER_BASE_H

#include <boost/optional.hpp>
#include <vector>
#include <xmlrpcpp/XmlRpcValue.h>
#include <Eigen/Dense>

namespace reach_plugins
{
namespace ik
{

/**
 * @brief Base class solving IK at a given reach study location
 */
class IKSolverBase
{
public:

  IKSolverBase()
  {

  }

  virtual ~IKSolverBase()
  {

  }

  /**
   * @brief initialize
   * @param config
   * @return
   */
  virtual bool initialize(XmlRpc::XmlRpcValue& config) = 0;

  /**
   * @brief solveIKFromSeed attempts to find a valid IK solution for the given target pose starting from the input seed state.
   * If a solution is found, the resulting IK solution is saved, and the pose is scored according to the specified cost function plugin
   * @param seed
   * @param solution
   * @return a boost optional type indicating the success of the IK solution and containing the score of the solution
   */
  virtual boost::optional<double> solveIKFromSeed(const Eigen::Affine3d& target,
                                                  const std::map<std::string, double>& seed,
                                                  std::vector<double>& solution) = 0;

};
typedef boost::shared_ptr<IKSolverBase> IKSolverBasePtr;

} // namespace ik
} // namespace reach_plugins

#endif // REACH_PLUGINS_IK_IK_SOLVER_BASE_H
