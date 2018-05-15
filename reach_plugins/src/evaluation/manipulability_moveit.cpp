#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_model/joint_model_group.h>
#include <reach_plugins/evaluation/manipulability_moveit.h>
#include <xmlrpcpp/XmlRpcException.h>

namespace reach_plugins
{
namespace evaluation
{

ManipulabilityMoveIt::ManipulabilityMoveIt()
  : EvaluationBase()
{

}

bool ManipulabilityMoveIt::initialize(XmlRpc::XmlRpcValue& config)
{
  if(!config.hasMember("planning_group"))
  {
    ROS_ERROR("MoveIt Manipulability Evaluation Plugin is missing 'planning_group' parameter");
    return false;
  }

  std::string planning_group;
  try
  {
    planning_group = std::string(config["planning_group"]);
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
    ROS_ERROR("Failed to initialize joint model group pointer");
    return false;
  }

  return true;
}

double ManipulabilityMoveIt::calculateScore(const std::vector<double>& pose)
{
  // Calculate manipulability of kinematic chain of input robot pose
  moveit::core::RobotState state(model_);
  state.setJointGroupPositions(jmg_, pose);
  state.update();

  // Get the Jacobian matrix
  Eigen::MatrixXd jacobian = state.getJacobian(jmg_);

  // Calculate manipulability by multiplying Jacobian matrix singular values together
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  Eigen::MatrixXd singular_values = svd.singularValues();
  double m = 1.0;
  for(unsigned int i = 0; i < singular_values.rows(); ++i)
  {
    m *= singular_values(i, 0);
  }
  return m;
}

} // namespace evaluation
} // namespace reach_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(reach_plugins::evaluation::ManipulabilityMoveIt, reach_plugins::evaluation::EvaluationBase)
