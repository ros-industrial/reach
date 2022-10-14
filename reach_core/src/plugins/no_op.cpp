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
    return boost::make_shared<NoOpEvaluator>();
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
    return boost::make_shared<NoOpIKSolver>();
  }
};

struct NoOpDisplay : public Display
{
  void showEnvironment() const override{};
  void updateRobotPose(const std::map<std::string, double>&) const override{};
  void showReachNeighborhood(const std::vector<ReachRecord>&) const override{};
  void showResults(const ReachDatabase&) const override{};
};

struct NoOpDisplayFactory : public DisplayFactory
{
  Display::ConstPtr create(const YAML::Node&) const override
  {
    return boost::make_shared<NoOpDisplay>();
  }
};

}  // namespace reach

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(reach::NoOpEvaluatorFactory, reach::EvaluatorFactory)
PLUGINLIB_EXPORT_CLASS(reach::NoOpIKSolverFactory, reach::IKSolverFactory)
PLUGINLIB_EXPORT_CLASS(reach::NoOpDisplayFactory, reach::DisplayFactory)
