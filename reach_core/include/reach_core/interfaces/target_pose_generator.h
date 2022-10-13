#pragma once

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
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
  using Ptr = boost::shared_ptr<TargetPoseGenerator>;
  using ConstPtr = boost::shared_ptr<const TargetPoseGenerator>;

  virtual VectorIsometry3d generate() const = 0;
};

struct TargetPoseGeneratorFactory
{
  using Ptr = boost::shared_ptr<TargetPoseGeneratorFactory>;
  using ConstPtr = boost::shared_ptr<const TargetPoseGeneratorFactory>;

  TargetPoseGeneratorFactory() = default;
  virtual ~TargetPoseGeneratorFactory() = default;

  virtual TargetPoseGenerator::ConstPtr create(const YAML::Node& config) const = 0;
};

} // namepsace reach
