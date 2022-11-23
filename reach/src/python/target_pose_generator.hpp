#pragma once

#include <reach/interfaces/target_pose_generator.h>
#include "utils.h"

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <yaml-cpp/yaml.h>

namespace bp = boost::python;
namespace np = boost::python::numpy;

namespace reach
{
TargetPoseGenerator::ConstPtr TargetPoseGeneratorFactory::create(const bp::dict& pyyaml_config) const
{
  return create(pythonDictToYAML(pyyaml_config));
}

struct TargetPoseGeneratorPython : TargetPoseGenerator, boost::python::wrapper<TargetPoseGenerator>
{
  VectorIsometry3d generateFunc() const
  {
    VectorIsometry3d eigen_list;

    bp::list np_list = this->get_override("generate")();

    // Convert the list of 4x4 numpy arrays to VectorIsometry3d
    for (int i = 0; i < bp::len(np_list); ++i)
    {
      Eigen::Isometry3d eigen_mat;
      bp::numpy::ndarray np_array = bp::numpy::from_object(np_list[i]);

      for (int j = 0; j < 4; ++j)
      {
        for (int k = 0; k < 4; ++k)
        {
          eigen_mat.matrix()(j, k) = bp::extract<double>(np_array[j][k]);
        }
      }
      eigen_list.push_back(eigen_mat);
    }

    return eigen_list;
  }
  VectorIsometry3d generate() const override
  {
    return call_and_handle(&TargetPoseGeneratorPython::generateFunc, this, "generate()");
  }
};

struct TargetPoseGeneratorFactoryPython : TargetPoseGeneratorFactory, boost::python::wrapper<TargetPoseGeneratorFactory>
{
  TargetPoseGenerator::ConstPtr createFunc(const YAML::Node& config) const
  {
    return this->get_override("create")(config);
  }
  TargetPoseGenerator::ConstPtr create(const YAML::Node& config) const override
  {
    return call_and_handle(&TargetPoseGeneratorFactoryPython::createFunc, this, "TargetPoseGeneratorFactory::create",
                           config);
  }
};

}  // namespace reach
