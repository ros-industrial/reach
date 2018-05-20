#ifndef REACH_PLUGINS_IK_DISCRETIZED_MOVEIT_IK_SOLVER_H
#define REACH_PLUGINS_IK_DISCRETIZED_MOVEIT_IK_SOLVER_H

#include "moveit_ik_solver.h"

namespace reach_plugins
{
namespace ik
{

class DiscretizedMoveItIKSolver : public MoveItIKSolver
{
public:

  DiscretizedMoveItIKSolver();

  virtual bool initialize(XmlRpc::XmlRpcValue& config) override;

  virtual boost::optional<double> solveIKFromSeed(const Eigen::Affine3d& target,
                                                  const std::vector<double>& seed,
                                                  std::vector<double>& solution) override;
protected:

  double dt_;
};

} // namespace ik
} // namespace reach_plugins

#endif // REACH_PLUGINS_IK_DISCRETIZED_MOVEIT_IK_SOLVER_H
