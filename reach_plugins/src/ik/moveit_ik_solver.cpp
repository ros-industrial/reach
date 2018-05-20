#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/PlanningScene.h>
#include <pluginlib/class_loader.h>
#include <reach_plugins/ik/moveit_ik_solver.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <xmlrpcpp/XmlRpcException.h>

namespace reach_plugins
{
namespace ik
{

MoveItIKSolver::MoveItIKSolver()
  : IKSolverBase()
{

}

bool MoveItIKSolver::initialize(XmlRpc::XmlRpcValue& config)
{
  if(!config.hasMember("planning_group") ||
     !config.hasMember("distance_threshold") ||
     !config.hasMember("evaluation_plugin"))
  {
    ROS_ERROR("MoveIt IK Solver Plugin is missing one or more configuration parameters");
    return false;
  }

  std::string planning_group;
  try
  {
    planning_group = std::string(config["planning_group"]);
    distance_threshold_ = double(config["distance_threshold"]);

    const static std::string PACKAGE = "reach_plugins";
    const static std::string EVAL_PLUGIN_BASE = "reach_plugins::evaluation::EvaluationBase";
    pluginlib::ClassLoader<evaluation::EvaluationBase> loader(PACKAGE, EVAL_PLUGIN_BASE);

    try
    {
      eval_ = loader.createInstance(config["evaluation_plugin"]["name"]);
    }
    catch(const pluginlib::ClassLoaderException& ex)
    {
      ROS_ERROR_STREAM(ex.what());
    }

    if(!eval_->initialize(config["evaluation_plugin"]))
    {
      ROS_ERROR_STREAM("Failed to initialize evaluation plugin");
      return false;
    }
  }
  catch(const XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR_STREAM(ex.getMessage());
    return false;
  }

  model_ = moveit::planning_interface::getSharedRobotModel("robot_description");
  if(!model_)
  {
    ROS_ERROR("Failed to initialize robot model pointer");
    return false;
  }

  jmg_ = model_->getJointModelGroup(planning_group);
  if(!jmg_)
  {
    ROS_ERROR_STREAM("Failed to get joint model group for '" << planning_group << "'");
    return false;
  }

  scene_.reset(new planning_scene::PlanningScene (model_));

  planning_scene_sub_ = nh_.subscribe("update_planning_scene", 1, &MoveItIKSolver::updatePlanningScene, this);

  ROS_INFO_STREAM("Successfully initialized MoveItIKSolver plugin");
  return true;
}

boost::optional<double> MoveItIKSolver::solveIKFromSeed(const Eigen::Affine3d& target,
                                                        const std::vector<double> &seed,
                                                        std::vector<double> &solution)
{
  moveit::core::RobotState goal_state (model_);
  moveit::core::RobotState seed_state (model_);

  assert(jmg_->getActiveJointModelNames().size() == seed.size());
  seed_state.setJointGroupPositions(jmg_, seed);
  seed_state.update();

  const static int SOLUTION_ATTEMPTS = 1;
  const static double SOLUTION_TIMEOUT = 0.02;

  if(goal_state.setFromIK(jmg_, target, SOLUTION_ATTEMPTS, SOLUTION_TIMEOUT, boost::bind(&MoveItIKSolver::isIKSolutionValid, this, _1, _2, _3)))
  {
    solution.clear();
    goal_state.copyJointGroupPositions(jmg_, solution);
    return eval_->calculateScore(solution);
  }
  else
  {
    return {};
  }
}

void MoveItIKSolver::updatePlanningScene(const moveit_msgs::PlanningSceneConstPtr& msg)
{
  if(!scene_->setPlanningSceneDiffMsg(*msg))
  {
    ROS_ERROR_STREAM("MoveItIKSolver failed to update planning scene");
  }
}

bool MoveItIKSolver::isIKSolutionValid(moveit::core::RobotState* state,
                                       const moveit::core::JointModelGroup* jmg,
                                       const double* ik_solution) const
{
  state->setJointGroupPositions(jmg, ik_solution);
  state->update();

  const bool colliding = scene_->isStateColliding(*state, jmg->getName(), false);
  const bool too_close = (scene_->distanceToCollision(*state, scene_->getAllowedCollisionMatrix()) > distance_threshold_);

  return (!colliding && !too_close);
}

} // namespace ik
} // namespace reach_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(reach_plugins::ik::MoveItIKSolver, reach_plugins::ik::IKSolverBase)
