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
#include <xmlrpcpp/XmlRpcValue.h>
#include <pcl/point_types_conversion.h>

namespace reach
{
namespace plugins
{
struct DisplayBase
{
  using Ptr = boost::shared_ptr<DisplayBase>;
  DisplayBase() = default;
  virtual ~DisplayBase() = default;

  virtual void initialize(XmlRpc::XmlRpcValue& config) = 0;

  virtual void showEnvironment() = 0;

  virtual void updateRobotPose(const std::map<std::string, double>& pose) = 0;

  virtual void compareDatabases(const std::map<std::string, reach::core::ReachDatabase>& dbs) = 0;
};

}  // namespace plugins
}  // namespace reach

#endif  // REACH_CORE_PLUGINS_DISPLAY_DISPLAY_BASE_H
