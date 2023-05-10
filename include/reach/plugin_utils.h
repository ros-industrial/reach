#ifndef reach_PLUGIN_UTILS_H
#define reach_PLUGIN_UTILS_H

#include <yaml-cpp/yaml.h>
#include <boost/core/demangle.hpp>
#include <Eigen/Geometry>

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

/**
 * @brief Attempts to resolve a prefix URI in a file name
 * @detials Supported URIs are `file://` and `package://`
 * @return The fully resolved filename
 */
std::string resolveURI(const std::string filename);

/**
 * @brief Creates a 6-DOF pose from an origin and normal vector, aligning the pose x-axis as closely as possible with
 * the unit x-axis
 */
Eigen::Isometry3d createFrame(const Eigen::Vector3f& pt, const Eigen::Vector3f& norm);

}  // namespace reach

#include <boost_plugin_loader/macros.h>
#define EXPORT_DISPLAY_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, DISPLAY_SECTION)
#define EXPORT_EVALUATOR_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, EVALUATOR_SECTION)
#define EXPORT_IK_SOLVER_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, IK_SOLVER_SECTION)
#define EXPORT_LOGGER_PLUGIN(DERIVED_CLASS, ALIAS) EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, LOGGER_SECTION)
#define EXPORT_TARGET_POSE_GENERATOR_PLUGIN(DERIVED_CLASS, ALIAS)                                                      \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, TARGET_POSE_GEN_SECTION)

#endif  // reach_PLUGIN_UTILS_H
