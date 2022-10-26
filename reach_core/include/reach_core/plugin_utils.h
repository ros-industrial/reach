#ifndef REACH_CORE_PLUGIN_UTILS_H
#define REACH_CORE_PLUGIN_UTILS_H

#include <yaml-cpp/yaml.h>
#include <boost/core/demangle.hpp>

namespace reach
{
/**
 * @brief Helper function for getting YAML parameters
 */
template <typename T>
T get(const YAML::Node& config, const std::string& key)
{
  if (!config[key].IsDefined())
  {
    std::stringstream ss;
    ss << "Failed to get '" << key << "' parameter within node at line " << config.Mark().line;
    throw std::runtime_error(ss.str());
  }

  try
  {
    return config[key].as<T>();
  }
  catch (const YAML::Exception& ex)
  {
    std::stringstream ss;
    ss << "Failed to cast '" << key << "' parameter with value '" << config[key] << "' to expected type '"
       << boost::core::demangle(typeid(T).name()) << "' (line " << ex.mark.line << ")";
    throw std::runtime_error(ss.str());
  }
}
}  // namespace reach

#include <boost_plugin_loader/macros.h>
#define EXPORT_DISPLAY_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, "disp")
#define EXPORT_EVALUATOR_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, "eval")
#define EXPORT_IK_SOLVER_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, "ik")
#define EXPORT_LOGGER_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, "logger")
#define EXPORT_TARGET_POSE_GENERATOR_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, "pose")

#endif  // REACH_CORE_PLUGIN_UTILS_H
