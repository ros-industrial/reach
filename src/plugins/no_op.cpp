#include <reach/plugins/no_op.h>

namespace reach
{
double NoOpEvaluator::calculateScore(const std::map<std::string, double>&) const
{
  return 0.0;
}

Evaluator::ConstPtr NoOpEvaluatorFactory::create(const YAML::Node&) const
{
  return std::make_shared<NoOpEvaluator>();
}

std::vector<std::string> NoOpIKSolver::getJointNames() const
{
  return { "j1" };
}

std::vector<std::vector<double>> NoOpIKSolver::solveIK(const Eigen::Isometry3d&,
                                                       const std::map<std::string, double>&) const
{
  return { { 0.0 } };
}

IKSolver::ConstPtr NoOpIKSolverFactory::create(const YAML::Node&) const
{
  return std::make_shared<NoOpIKSolver>();
}

void NoOpDisplay::showEnvironment() const
{
}
void NoOpDisplay::updateRobotPose(const std::map<std::string, double>&) const
{
}
void NoOpDisplay::showReachNeighborhood(const std::map<std::size_t, ReachRecord>&) const
{
}
void NoOpDisplay::showResults(const ReachResult&) const
{
}

Display::ConstPtr NoOpDisplayFactory::create(const YAML::Node&) const
{
  return std::make_shared<NoOpDisplay>();
}

}  // namespace reach
