#pragma once

#include <reach/interfaces/ik_solver.h>
#include "utils.hpp"

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <yaml-cpp/yaml.h>

namespace bp = boost::python;

namespace reach
{
boost::python::list IKSolver::solveIK(const boost::python::numpy::ndarray& target,
                                      const boost::python::dict& seed) const
{
  Eigen::Isometry3d cpp_target = toEigen(target);
  std::map<std::string, double> cpp_seed = toMap<std::string, double>(seed);

  std::vector<std::vector<double>> cpp_solution = solveIK(cpp_target, cpp_seed);

  boost::python::list solution;
  for (const std::vector<double>& inner : cpp_solution)
  {
    solution.append(inner);
  }

  return solution;
}

IKSolver::ConstPtr IKSolverFactory::create(const boost::python::dict& pyyaml_config) const
{
  return create(toYAML(pyyaml_config));
}

struct IKSolverPython : IKSolver, boost::python::wrapper<IKSolver>
{
  std::vector<std::string> getJointNames() const override
  {
    auto fn = [this]() -> std::vector<std::string> {
      std::vector<std::string> names;

      bp::list name_list = this->get_override("getJointNames")();

      for (int i = 0; i < bp::len(name_list); ++i)
      {
        std::string name = bp::extract<std::string>{ name_list[i] }();
        names.push_back(name);
      }

      return names;
    };
    return call_and_handle(fn);
  }

  std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d& target,
                                           const std::map<std::string, double>& seed) const override
  {
    auto fn = [this, &target, &seed]() -> std::vector<std::vector<double>> {
      // Convert the seed to a Python dictionary
      bp::dict dictionary;
      for (const auto& pair : seed)
      {
        dictionary[pair.first] = pair.second;
      }

      bp::list list = this->get_override("solveIK")(fromEigen(target), dictionary);

      // Convert the output
      std::vector<std::vector<double>> output;
      for (int i = 0; i < bp::len(list); ++i)
      {
        std::vector<double> sub_vec;
        bp::list sub_list = bp::extract<bp::list>{ list[i] }();
        for (int j = 0; j < bp::len(sub_list); ++j)
        {
          sub_vec.push_back(bp::extract<double>{ sub_list[j] }());
        }
        output.push_back(sub_vec);
      }

      return output;
    };

    return call_and_handle(fn);
  }
};

struct IKSolverFactoryPython : IKSolverFactory, boost::python::wrapper<IKSolverFactory>
{
  IKSolver::ConstPtr create(const YAML::Node& config) const override
  {
    return call_and_handle([this, &config]() -> IKSolver::ConstPtr { return this->get_override("create")(config); });
  }
};

}  // namespace reach
