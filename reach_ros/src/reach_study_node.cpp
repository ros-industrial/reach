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
#include <reach/reach_study.h>
#include <reach_ros/utils.h>

#include <thread>
#include <chrono>

#include <boost/filesystem.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

template <typename T>
T get(const std::shared_ptr<rclcpp::Node> node, const std::string& key)
{
  T val;
  if (!node->get_parameter(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

namespace reach_ros
{
namespace utils
{
  // we need to do this since the node is specified as "extern" in the shared library
  rclcpp::Node::SharedPtr node;
}
}

int main(int argc, char** argv)
{
  try
  {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor = rclcpp::executors::MultiThreadedExecutor();
    std::thread executor_thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor));
    reach_ros::utils::node = std::make_shared<rclcpp::Node>("reach_study_node", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));
    executor.add_node(reach_ros::utils::node);

    // Load the configuration information
    const YAML::Node config = YAML::LoadFile(get<std::string>(reach_ros::utils::getNodeInstance(), "config_file"));
    const std::string config_name = get<std::string>(reach_ros::utils::getNodeInstance(), "config_name");
    const boost::filesystem::path results_dir(get<std::string>(reach_ros::utils::getNodeInstance(), "results_dir"));

    // Run the reach study
    reach::runReachStudy(config, config_name, results_dir, true);
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
  }

  return 0;
}
