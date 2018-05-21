#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <reach_plugins/display/reach_display_base.h>
#include <reach_plugins/ik/ik_solver_base.h>
#include <xmlrpcpp/XmlRpcException.h>

template<typename T>
bool loadPlugin(XmlRpc::XmlRpcValue& config,
                pluginlib::ClassLoader<T>& loader,
                boost::shared_ptr<T>& plugin)
{
  std::string plugin_name;
  try
  {
    plugin_name = std::string(config["name"]);
  }
  catch(const XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR_STREAM(ex.getMessage());
    return false;
  }

  try
  {
    plugin = loader.createInstance(plugin_name);
  }
  catch(const pluginlib::ClassLoaderException& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }

  if(!plugin->initialize(config))
  {
    ROS_ERROR_STREAM("Failed to initialize plugin");
    return false;
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plugin_test_node");
  ros::NodeHandle pnh("~");

  XmlRpc::XmlRpcValue display_config;
  if(!pnh.getParam("reach_display", display_config))
  {
    ROS_ERROR_STREAM("Failed to get 'display_config' parameter");
    return -1;
  }

  reach_plugins::display::ReachDisplayBasePtr display_plugin;
  pluginlib::ClassLoader<reach_plugins::display::ReachDisplayBase> display_loader ("reach_plugins", "reach_plugins::display::ReachDisplayBase");
  if(!loadPlugin<reach_plugins::display::ReachDisplayBase>(display_config, display_loader, display_plugin))
  {
    ROS_ERROR("Failed to load reach display plugin");
    return -1;
  }
  display_plugin->showEnvironment();

  XmlRpc::XmlRpcValue ik_solver_config;
  if(!pnh.getParam("ik_solver", ik_solver_config))
  {
    ROS_ERROR_STREAM("Failed to get 'ik_solver' parameter");
    return -1;
  }

  reach_plugins::ik::IKSolverBasePtr ik_solver_plugin;
  pluginlib::ClassLoader<reach_plugins::ik::IKSolverBase> ik_solver_loader ("reach_plugins", "reach_plugins::ik::IKSolverBase");
  if(!loadPlugin<reach_plugins::ik::IKSolverBase>(ik_solver_config, ik_solver_loader, ik_solver_plugin))
  {
    ROS_ERROR("Failed to load IK solver plugin");
    return -1;
  }

  ros::spin();

  return 0;
}
