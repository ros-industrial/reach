#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>
#include <reach_plugins/ik/discretized_moveit_ik_solver.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <algorithm>

namespace
{

template<typename T>
T clamp(const T& val,
        const T& low,
        const T& high)
{
  return std::max(low, std::min(val, high));
}

} // namespace anonymous

namespace reach_plugins
{
namespace ik
{

DiscretizedMoveItIKSolver::DiscretizedMoveItIKSolver()
  : MoveItIKSolver()
{

}

bool DiscretizedMoveItIKSolver::initialize(XmlRpc::XmlRpcValue& config)
{
  if(!MoveItIKSolver::initialize(config))
  {
    ROS_ERROR("Failed to initialize MoveItIKSolver plugin");
    return false;
  }

  try
  {
    dt_ = std::abs(double(config["discretization_angle"]));
    double clamped_dt = clamp<double>(dt_, 0.0f, M_PI);
    if(dt_ != clamped_dt)
    {
      ROS_WARN_STREAM("Clamping discretization angle between 0 and pi; new value is " << clamped_dt);
    }
    dt_ = clamped_dt;
  }
  catch(const XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR_STREAM(ex.getMessage());
    return false;
  }

  ROS_INFO_STREAM("Successfully initialized DiscretizedMoveItIKSolver plugin");
  return true;
}

boost::optional<double> DiscretizedMoveItIKSolver::solveIKFromSeed(const Eigen::Affine3d& target,
                                                                   const std::vector<double>& seed,
                                                                   std::vector<double>& solution)
{
  // Calculate the number of discretizations necessary to achieve discretization angle
  const static int n_discretizations = int((2.0*M_PI) / dt_);

  // Set up containers for the best solution to be saved into the database
  std::vector<double> best_solution;
  double best_score = 0;

  for(int i = 0; i < n_discretizations; ++i)
  {
    Eigen::Affine3d discretized_target (target * Eigen::AngleAxisd (double(i)*dt_, Eigen::Vector3d::UnitZ()));
    std::vector<double> tmp_solution;

    boost::optional<double> score = MoveItIKSolver::solveIKFromSeed(discretized_target, seed, tmp_solution);
    if(score && (score.get() > best_score))
    {
      best_score = *score;
      best_solution = std::move(tmp_solution);
    }
    else
    {
      continue;
    }
  }

  if(best_score > 0)
  {
    solution = std::move(best_solution);
    return boost::optional<double>(best_score);
  }
  else
  {
    return {};
  }
}

} // namespace ik
} // namespace reach_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(reach_plugins::ik::DiscretizedMoveItIKSolver, reach_plugins::ik::IKSolverBase)
