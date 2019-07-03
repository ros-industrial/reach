#include <pluginlib/class_loader.h>
#include <reach_plugins/evaluation/multiplicative_factory.h>
#include <ros/console.h>
#include <xmlrpcpp/XmlRpcException.h>

namespace reach_plugins
{
namespace evaluation
{

MultiplicativeFactory::MultiplicativeFactory()
  : EvaluationBase()
{

}

bool MultiplicativeFactory::initialize(XmlRpc::XmlRpcValue& config)
{
  try
  {
    XmlRpc::XmlRpcValue& plugin_configs = config["plugins"];

    eval_plugins_.reserve(plugin_configs.size());

    const static std::string PACKAGE = "reach_plugins";
    const static std::string PLUGIN_BASE_NAME = "reach_plugins::evaluation::EvaluationBase";
    pluginlib::ClassLoader<EvaluationBase> loader (PACKAGE, PLUGIN_BASE_NAME);

    for(int i = 0; i < plugin_configs.size(); ++i)
    {
      XmlRpc::XmlRpcValue& plugin_config = plugin_configs[i];
      const std::string name = std::string(plugin_config["name"]);

      EvaluationBasePtr plugin;
      try
      {
        plugin = loader.createInstance(name);
      }
      catch(const pluginlib::ClassLoaderException& ex)
      {
        ROS_WARN_STREAM("Plugin '" << name << "' failed to load: " << ex.what() << "; excluding it from the list");
        continue;
      }

      if(!plugin->initialize(plugin_config))
      {
        ROS_WARN_STREAM("Plugin '" << name << "' failed to be initialized; excluding it from the list");
        continue;
      }

      eval_plugins_.push_back(std::move(plugin));
    }
  }
  catch(const XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR_STREAM(ex.getMessage());
  }

  if(eval_plugins_.empty())
  {
    ROS_ERROR("No valid plugins remain");
    return false;
  }

  return true;
}

double MultiplicativeFactory::calculateScore(const std::map<std::string, double>& pose)
{
  double score = 1.0;
  for(const EvaluationBasePtr& plugin : eval_plugins_)
  {
    score *= plugin->calculateScore(pose);
  }
  return score;
}

} // namespace evaluation
} // namespace reach_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(reach_plugins::evaluation::MultiplicativeFactory, reach_plugins::evaluation::EvaluationBase)
