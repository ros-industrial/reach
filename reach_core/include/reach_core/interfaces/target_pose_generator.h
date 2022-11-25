#ifndef REACH_CORE_INTERFACES_TARGET_POSE_GENERATOR_H
#define REACH_CORE_INTERFACES_TARGET_POSE_GENERATOR_H

#include <reach_core/reach_database.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace YAML
{
class Node;
}

namespace reach
{
/** @brief Interface for generating Cartesian target poses for the reach study */
struct TargetPoseGenerator
{
  using Ptr = std::shared_ptr<TargetPoseGenerator>;
  using ConstPtr = std::shared_ptr<const TargetPoseGenerator>;

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
    return "pose";
  }
};

}  // namespace reach

#endif  // REACH_CORE_INTERFACES_TARGET_POSE_GENERATOR_H
