#ifndef reach_INTERFACES_TARGET_POSE_GENERATOR_H
#define reach_INTERFACES_TARGET_POSE_GENERATOR_H

#include <reach/types.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace YAML
{
class Node;
}

#ifdef BUILD_PYTHON
namespace boost
{
namespace python
{
class dict;
}  // namespace python
}  // namespace boost
#endif

namespace reach
{
/** @brief Interface for generating Cartesian target poses for the reach study */
struct TargetPoseGenerator
{
  using Ptr = std::shared_ptr<TargetPoseGenerator>;
  using ConstPtr = std::shared_ptr<const TargetPoseGenerator>;

  virtual ~TargetPoseGenerator() = default;

  /**
   * @brief Creates a list of Cartesian target poses for the reach study
   * @details The Cartesian poses should be relative to the robot kinematic base frame, and the pose z-axis should
   * oppose the orientation of the robot tool z-axis
   */
  virtual VectorIsometry3d generate() const = 0;
};

/** @brief Plugin interface for creating target pose generator interfaces */
struct TargetPoseGeneratorFactory
{
  using Ptr = std::shared_ptr<TargetPoseGeneratorFactory>;
  using ConstPtr = std::shared_ptr<const TargetPoseGeneratorFactory>;

  TargetPoseGeneratorFactory() = default;
  virtual ~TargetPoseGeneratorFactory() = default;

  virtual TargetPoseGenerator::ConstPtr create(const YAML::Node& config) const = 0;

  static std::string getSection()
  {
    return TARGET_POSE_GEN_SECTION;
  }

#ifdef BUILD_PYTHON
  TargetPoseGenerator::ConstPtr create(const boost::python::dict& pyyaml_config) const;
#endif
};

}  // namespace reach

#endif  // reach_INTERFACES_TARGET_POSE_GENERATOR_H
