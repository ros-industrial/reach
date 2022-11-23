#pragma once

#include <reach/interfaces/display.h>
#include "utils.h"

#include <boost/python.hpp>
#include <yaml-cpp/yaml.h>

namespace bp = boost::python;

namespace reach
{
void Display::updateRobotPose(const bp::dict& dict_joint_positions) const
{
  return updateRobotPose(pythonDictToMap<std::string, double>(dict_joint_positions));
}

Display::ConstPtr DisplayFactory::create(const bp::dict& pyyaml_config) const
{
  return create(pythonDictToYAML(pyyaml_config));
}

struct DisplayPython : Display, boost::python::wrapper<Display>
{
  void showEnvironmentFunc() const
  {
    this->get_override("showEnvironment")();
  }
  void showEnvironment() const override
  {
    return call_and_handle(&DisplayPython::showEnvironmentFunc, this, "showEnvironment()");
  }

  void updateRobotPoseFunc(const std::map<std::string, double>& map) const
  {
    bp::dict dictionary;
    for (auto pair : map)
    {
      dictionary[pair.first] = pair.second;
    }

    this->get_override("updateRobotPose")(dictionary);
  }
  void updateRobotPose(const std::map<std::string, double>& map) const override
  {
    return call_and_handle(&DisplayPython::updateRobotPoseFunc, this, "updateRobotPose()", map);
  }

  void showReachNeighborhoodFunc(const std::vector<ReachRecord>& neighborhood) const
  {
    this->get_override("showReachNeighborhood")(neighborhood);
  }
  void showReachNeighborhood(const std::vector<ReachRecord>& neighborhood) const override
  {
    return call_and_handle(&DisplayPython::showReachNeighborhoodFunc, this, "showReachNeighborhood()", neighborhood);
  }

  void showResultsFunc(const ReachDatabase& results) const
  {
    this->get_override("showResults")(results);
  }
  void showResults(const ReachDatabase& results) const override
  {
    return call_and_handle(&DisplayPython::showResultsFunc, this, "showResults()", results);
  }
};

struct DisplayFactoryPython : DisplayFactory, boost::python::wrapper<DisplayFactory>
{
  Display::ConstPtr createFunc(const YAML::Node& config) const
  {
    return this->get_override("create")(config);
  }
  Display::ConstPtr create(const YAML::Node& config) const override
  {
    return call_and_handle(&DisplayFactoryPython::createFunc, this, "DisplayFactory::create", config);
  }
};

}  // namespace reach
