#pragma once

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <xmlrpcpp/XmlRpcValue.h>

namespace reach
{
namespace plugins
{
using VectorIsometry3d = std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;

struct TargetPoseGeneratorBase
{
  using Ptr = boost::shared_ptr<TargetPoseGeneratorBase>;
  virtual VectorIsometry3d generate() const = 0;

  virtual void initialize(const XmlRpc::XmlRpcValue& config) = 0;
};

} // namespace core
} // namepsace reach
