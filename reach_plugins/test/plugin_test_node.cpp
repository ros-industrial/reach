#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <reach_plugins/ik/ik_solver_base.h>
#include <xmlrpcpp/XmlRpcException.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plugin_test_node");
  ros::NodeHandle pnh("~");

  XmlRpc::XmlRpcValue config;
  if(!pnh.getParam("config", config))
  {
    ROS_ERROR_STREAM("Failed to get 'config' parameter");
    return -1;
  }

  pluginlib::ClassLoader<reach_plugins::ik::IKSolverBase> loader ("reach_plugins", "reach_plugins::ik::IKSolverBase");

  std::string plugin_name;
  try
  {
    plugin_name = std::string(config["name"]);
  }
  catch(const XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR_STREAM(ex.getMessage());
    return -1;
  }

  reach_plugins::ik::IKSolverBasePtr solver;
  try
  {
    solver = loader.createInstance(plugin_name);
  }
  catch(const pluginlib::ClassLoaderException& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  if(!solver->initialize(config))
  {
    ROS_ERROR_STREAM("Failed to initialize IK plugin");
    return -1;
  }

  return 0;
}
