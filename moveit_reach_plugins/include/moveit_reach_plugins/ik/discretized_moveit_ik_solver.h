#ifndef MOVEIT_REACH_PLUGINS_IK_DISCRETIZED_MOVEIT_IK_SOLVER_H
#define MOVEIT_REACH_PLUGINS_IK_DISCRETIZED_MOVEIT_IK_SOLVER_H

#include "moveit_ik_solver.h"

namespace moveit_reach_plugins
{
namespace ik
{

class DiscretizedMoveItIKSolver : public MoveItIKSolver
{
public:

  DiscretizedMoveItIKSolver();

  virtual bool initialize(XmlRpc::XmlRpcValue& config) override;

  virtual boost::optional<double> solveIKFromSeed(const Eigen::Affine3d& target,
                                                  const std::map<std::string, double>& seed,
                                                  std::vector<double>& solution) override;

protected:

  double dt_;
};

} // namespace ik
} // namespace moveit_reach_plugins

#endif // MOVEIT_REACH_PLUGINS_IK_DISCRETIZED_MOVEIT_IK_SOLVER_H
