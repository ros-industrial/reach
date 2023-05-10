#pragma once

#include <reach/interfaces/display.h>
#include "utils.hpp"

#include <boost/python.hpp>
#include <yaml-cpp/yaml.h>

namespace bp = boost::python;

namespace reach
{
void Display::updateRobotPose(const bp::dict& dict_joint_positions) const
{
  return updateRobotPose(toMap<std::string, double>(dict_joint_positions));
}

Display::ConstPtr DisplayFactory::create(const bp::dict& pyyaml_config) const
{
  return create(toYAML(pyyaml_config));
}

struct DisplayPython : Display, boost::python::wrapper<Display>
{
  void showEnvironment() const override
  {
    return call_and_handle([this]() { this->get_override("showEnvironment")(); });
  }

  void updateRobotPose(const std::map<std::string, double>& map) const override
  {
    auto fn = [this, &map]() {
      bp::dict dictionary;
      for (auto pair : map)
      {
        dictionary[pair.first] = pair.second;
      }

      this->get_override("updateRobotPose")(dictionary);
    };

    return call_and_handle(fn);
  }

  void showReachNeighborhood(const std::map<std::size_t, ReachRecord>& neighborhood) const override
  {
    return call_and_handle([this, &neighborhood]() { this->get_override("showReachNeighborhood")(neighborhood); });
  }

  void showResults(const ReachResult& results) const override
  {
    return call_and_handle([this, &results]() { this->get_override("showResults")(results); });
  }
};

struct DisplayFactoryPython : DisplayFactory, boost::python::wrapper<DisplayFactory>
{
  Display::ConstPtr create(const YAML::Node& config) const override
  {
    return call_and_handle([this, &config]() -> Display::ConstPtr { return this->get_override("create")(config); });
  }
};

}  // namespace reach
