#pragma once

#include <reach/interfaces/ik_solver.h>
#include "utils.h"

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <yaml-cpp/yaml.h>

namespace bp = boost::python;

namespace reach
{
boost::python::list IKSolver::solveIK(const boost::python::numpy::ndarray& target,
                                      const boost::python::dict& seed) const
{
  Eigen::Isometry3d cpp_target;
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      cpp_target.matrix()(i, j) = boost::python::extract<double>(target[i, j]);
    }
  }

  std::map<std::string, double> cpp_seed = pythonDictToMap<std::string, double>(seed);

  std::vector<std::vector<double>> cpp_solution = solveIK(cpp_target, cpp_seed);

  boost::python::list solution;
  for (std::vector<double> inner : cpp_solution)
  {
    solution.append(inner);
  }

  return solution;
}

IKSolver::ConstPtr IKSolverFactory::create(const boost::python::dict& pyyaml_config) const
{
  return create(pythonDictToYAML(pyyaml_config));
}

struct IKSolverPython : IKSolver, boost::python::wrapper<IKSolver>
{
  std::vector<std::string> getJointNamesFunc() const
  {
    std::vector<std::string> names;

    bp::list name_list = this->get_override("getJointNames")();

    for (int i = 0; i < bp::len(name_list); ++i)
    {
      std::string name = bp::extract<std::string>(name_list[i]);
      names.push_back(name);
    }

    return names;
  }
  std::vector<std::string> getJointNames() const override
  {
    return call_and_handle(&IKSolverPython::getJointNamesFunc, this, "getJointNames()");
  }

  std::vector<std::vector<double>> solveIKFunc(const Eigen::Isometry3d& target,
                                               const std::map<std::string, double>& seed) const
  {
    std::vector<std::vector<double>> output;

    bp::tuple shape = bp::make_tuple(4, 4);
    bp::numpy::dtype dtype = bp::numpy::dtype::get_builtin<double>();
    bp::numpy::ndarray array = bp::numpy::zeros(shape, dtype);

    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        array[i, j] = target.matrix()(i, j);
      }
    }

    bp::dict dictionary;
    for (std::pair<std::string, double> pair : seed)
    {
      dictionary[pair.first] = pair.second;
    }

    bp::list list = this->get_override("solveIK")(array, dictionary);

    for (int i = 0; i < bp::len(list); ++i)
    {
      std::vector<double> sub_vec;
      bp::list sub_list = bp::extract<bp::list>(list[i]);
      for (int j = 0; j < bp::len(sub_list); ++j)
      {
        sub_vec.push_back(bp::extract<double>(sub_list[j]));
      }
      output.push_back(sub_vec);
    }

    return output;
  }
  std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d& target,
                                           const std::map<std::string, double>& seed) const override
  {
    return call_and_handle(&IKSolverPython::solveIKFunc, this, "solveIK()", target, seed);
  }
};

struct IKSolverFactoryPython : IKSolverFactory, boost::python::wrapper<IKSolverFactory>
{
  IKSolver::ConstPtr createFunc(const YAML::Node& config) const
  {
    return this->get_override("create")(config);
  }
  IKSolver::ConstPtr create(const YAML::Node& config) const override
  {
    return call_and_handle(&IKSolverFactoryPython::createFunc, this, "IKSolverFactory::create", config);
  }
};

}  // namespace reach
