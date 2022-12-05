#pragma once

#include <reach/interfaces/target_pose_generator.h>
#include "utils.hpp"

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <yaml-cpp/yaml.h>

namespace bp = boost::python;
namespace np = boost::python::numpy;

namespace reach
{
TargetPoseGenerator::ConstPtr TargetPoseGeneratorFactory::create(const bp::dict& pyyaml_config) const
{
  return create(toYAML(pyyaml_config));
}

struct TargetPoseGeneratorPython : TargetPoseGenerator, boost::python::wrapper<TargetPoseGenerator>
{
  VectorIsometry3d generate() const override
  {
    auto fn = [this]() -> VectorIsometry3d {
      bp::list np_list = this->get_override("generate")();

      // Convert the list of 4x4 numpy arrays to VectorIsometry3d
      VectorIsometry3d eigen_list;
      for (int i = 0; i < bp::len(np_list); ++i)
      {
        eigen_list.push_back(toEigen(bp::numpy::from_object(np_list[i])));
      }

      return eigen_list;
    };

    return call_and_handle(fn);
  }
};

struct TargetPoseGeneratorFactoryPython : TargetPoseGeneratorFactory, boost::python::wrapper<TargetPoseGeneratorFactory>
{
  TargetPoseGenerator::ConstPtr create(const YAML::Node& config) const override
  {
    return call_and_handle(
        [this, &config]() -> TargetPoseGenerator::ConstPtr { return this->get_override("create")(config); });
  }
};

}  // namespace reach
