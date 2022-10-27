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
#include <reach_core/reach_study.h>

#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

template <typename T>
T get(const ros::NodeHandle& nh, const std::string& key)
{
  T val;
  if (!nh.getParam(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "reach_study_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle pnh("~");

    // Load the configuration information
    const YAML::Node config = YAML::LoadFile(get<std::string>(pnh, "config_file"));
    const std::string config_name = get<std::string>(pnh, "config_name");
    const boost::filesystem::path results_dir(get<std::string>(pnh, "results_dir"));

    // Run the reach study
    reach::runReachStudy(config, config_name, results_dir, true);
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
  }

  return 0;
}
