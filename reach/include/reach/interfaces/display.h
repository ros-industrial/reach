/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef reach_INTERFACES_DISPLAY_H
#define reach_INTERFACES_DISPLAY_H

#include <reach/types.h>

#include <memory>

namespace YAML
{
class Node;
}

#ifdef BUILD_PYTHON
namespace boost
{
namespace python
{
class dict;
}  // namespace python
}  // namespace boost
#endif

namespace reach
{
/**
 * @brief Interface for displaying the state and results of a reach study
 */
struct Display
{
  using Ptr = std::shared_ptr<Display>;
  using ConstPtr = std::shared_ptr<const Display>;

  Display() = default;
  virtual ~Display() = default;

  /** @brief Visualizes the geometry of the robot and its environment in the reach study */
  virtual void showEnvironment() const = 0;

  /**
   * @brief Updates the visualization to show the input robot pose
   * @param pose map of joint names to joint values
   */
  virtual void updateRobotPose(const std::map<std::string, double>& pose) const = 0;

  /** @brief Visualizes a collection of points that are reachable */
  virtual void showReachNeighborhood(const std::map<std::size_t, ReachRecord>& neighborhood) const = 0;

  /** @brief Visualizes the results of a reach study */
  virtual void showResults(const ReachResult& db) const = 0;

#ifdef BUILD_PYTHON
  void updateRobotPose(const boost::python::dict&) const;
#endif
};

/**
 * @brief Plugin interface for creating display interfaces
 */
struct DisplayFactory
{
  using Ptr = std::shared_ptr<DisplayFactory>;
  using ConstPtr = std::shared_ptr<const DisplayFactory>;

  DisplayFactory() = default;
  virtual ~DisplayFactory() = default;

  virtual Display::ConstPtr create(const YAML::Node& config) const = 0;

  static std::string getSection()
  {
    return DISPLAY_SECTION;
  }

#ifdef BUILD_PYTHON
  Display::ConstPtr create(const boost::python::dict& pyyaml_config) const;
#endif
};

}  // namespace reach

#endif  // reach_INTERFACES_DISPLAY_H
