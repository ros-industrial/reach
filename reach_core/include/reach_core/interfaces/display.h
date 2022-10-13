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
#ifndef REACH_CORE_PLUGINS_DISPLAY_DISPLAY_BASE_H
#define REACH_CORE_PLUGINS_DISPLAY_DISPLAY_BASE_H

#include <reach_core/reach_database.h>

#include <boost/shared_ptr.hpp>

namespace YAML
{
class Node;
}

namespace reach
{
struct Display
{
  using Ptr = boost::shared_ptr<Display>;
  using ConstPtr = boost::shared_ptr<const Display>;

  Display() = default;
  virtual ~Display() = default;

  virtual void showEnvironment() const = 0;
  virtual void updateRobotPose(const std::map<std::string, double>& pose) const = 0;
  virtual void showReachNeighborhood(const std::vector<ReachRecord>& neighborhood) const = 0;
  //  virtual void compareDatabases(const std::map<std::string, ReachDatabase>& dbs) const = 0;
};

struct DisplayFactory
{
  using Ptr = boost::shared_ptr<DisplayFactory>;
  using ConstPtr = boost::shared_ptr<const DisplayFactory>;

  DisplayFactory() = default;
  virtual ~DisplayFactory() = default;

  virtual Display::ConstPtr create(const YAML::Node& config) const = 0;
};

}  // namespace reach

#endif  // REACH_CORE_PLUGINS_DISPLAY_DISPLAY_BASE_H
