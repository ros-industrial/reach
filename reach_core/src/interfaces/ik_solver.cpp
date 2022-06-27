#include <reach_core/interfaces/ik_solver.h>
#include <reach_core/utils.h>

namespace reach
{
IKSolver::IKSolver() : loader_("", "")
{
}

void IKSolver::initialize(XmlRpc::XmlRpcValue& config)
{
  eval_ = loader_.createInstance(config["eval_plugin"]["name"]);
  eval_->initialize(config["eval_plugin"]);
  initializeImpl(config);
}

std::tuple<std::vector<double>, double> IKSolver::solveIKFromSeed(const Eigen::Isometry3d& target,
                                                                      const std::map<std::string, double>& seed) const
{
  std::vector<std::vector<double>> poses = solveIKFromSeedImpl(target, seed);
  const std::vector<std::string> joint_names = getJointNames();
  double best_score = 0.0;
  std::size_t best_idx = 0;

  for (std::size_t i = 0; i < poses.size(); ++i)
  {
    double score = eval_->calculateScore(zip(joint_names, poses[i]));
    if (score > best_score)
    {
      best_score = score;
      best_idx = i;
    }
  }

  return std::make_tuple(poses.at(best_idx), best_score);
}

} // namespace reach
