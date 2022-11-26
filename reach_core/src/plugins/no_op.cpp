#include <reach_core/interfaces/evaluator.h>
#include <reach_core/interfaces/ik_solver.h>
#include <reach_core/interfaces/display.h>

#include <boost/shared_ptr.hpp>

namespace reach
{
struct NoOpEvaluator : public Evaluator
{
  double calculateScore(const std::map<std::string, double>&) const override
  {
    return 0.0;
  }
};

struct NoOpEvaluatorFactory : public EvaluatorFactory
{
  virtual Evaluator::ConstPtr create(const YAML::Node&) const override
  {
    return std::make_shared<NoOpEvaluator>();
  }
};

struct NoOpIKSolver : public IKSolver
{
public:
  std::vector<std::string> getJointNames() const override
  {
    return { "j1" };
  }

  std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d&,
                                           const std::map<std::string, double>&) const override
  {
    return { { 0.0 } };
  }
};

struct NoOpIKSolverFactory : public IKSolverFactory
{
  IKSolver::ConstPtr create(const YAML::Node&) const override
  {
    return std::make_shared<NoOpIKSolver>();
  }
};

struct NoOpDisplay : public Display
{
  void showEnvironment() const override{};
  void updateRobotPose(const std::map<std::string, double>&) const override{};
  void showReachNeighborhood(const std::map<std::size_t, ReachRecord>&) const override{};
  void showResults(const ReachResult&) const override{};
};

struct NoOpDisplayFactory : public DisplayFactory
{
  Display::ConstPtr create(const YAML::Node&) const override
  {
    return std::make_shared<NoOpDisplay>();
  }
};

}  // namespace reach

#include <reach_core/plugin_utils.h>
EXPORT_EVALUATOR_PLUGIN(reach::NoOpEvaluatorFactory, NoOpEvaluator)
EXPORT_IK_SOLVER_PLUGIN(reach::NoOpIKSolverFactory, NoOpIKSolver)
EXPORT_DISPLAY_PLUGIN(reach::NoOpDisplayFactory, NoOpDisplay)
