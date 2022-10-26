#ifndef REACH_CORE_INTERFACES_TARGET_POSE_GENERATOR_H
#define REACH_CORE_INTERFACES_TARGET_POSE_GENERATOR_H

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace YAML
{
class Node;
}

namespace reach
{
using VectorIsometry3d = std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;

struct TargetPoseGenerator
{
  using Ptr = std::shared_ptr<TargetPoseGenerator>;
  using ConstPtr = std::shared_ptr<const TargetPoseGenerator>;

  virtual VectorIsometry3d generate() const = 0;
};

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

} // namepsace reach

#endif  // REACH_CORE_INTERFACES_TARGET_POSE_GENERATOR_H
