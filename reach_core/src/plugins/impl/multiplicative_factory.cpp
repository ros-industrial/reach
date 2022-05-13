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
#include "reach_core/plugins/impl/multiplicative_factory.h"

#include <rclcpp/rclcpp.hpp>

namespace reach {
namespace {
const rclcpp::Logger LOGGER =
    rclcpp::get_logger("reach.multiplicative_factory");
}

namespace plugins {

constexpr char PACKAGE[] = "reach_core";
constexpr char PLUGIN_BASE_NAME[] = "reach::plugins::EvaluationBase";

MultiplicativeFactory::MultiplicativeFactory()
    : EvaluationBase(), class_loader_(PACKAGE, PLUGIN_BASE_NAME) {}

bool MultiplicativeFactory::initialize(
    std::string &name, rclcpp::Node::SharedPtr const node,
    std::shared_ptr<const moveit::core::RobotModel> model) {
  try {
    std::vector<std::string> plugin_configs;
    node->get_parameter("ik_solver_config.evaluation_plugin.plugins",
                        plugin_configs);

    eval_plugins_.reserve(plugin_configs.size());

    for (int i = 0; i < plugin_configs.size(); ++i) {
      std::string &plugin_config = plugin_configs[i];
      const std::string plugin_name = std::string(plugin_config);
      RCLCPP_INFO(LOGGER, "Creating shared instance of plugin '%s'",
                  plugin_name.c_str());

      EvaluationBasePtr plugin;
      try {
        plugin = class_loader_.createSharedInstance(plugin_name);
      } catch (const pluginlib::ClassLoaderException &ex) {
        RCLCPP_WARN_STREAM(
            LOGGER, "Plugin '" << name << "' failed to load: " << ex.what()
                               << "; excluding it from the list");
        continue;
      }

      if (!plugin->initialize(name, node, model)) {
        RCLCPP_WARN_STREAM(
            LOGGER,
            "Plugin '"
                << name
                << "' failed to be initialized; excluding it from the list");
        continue;
      }

      eval_plugins_.push_back(std::move(plugin));
    }
  } catch (const std::exception &ex) {
    RCLCPP_ERROR_STREAM(LOGGER, ex.what());
  }

  if (eval_plugins_.empty()) {
    RCLCPP_ERROR(LOGGER, "No valid plugins remain");
    return false;
  }

  return true;
}

double MultiplicativeFactory::calculateScore(
    const std::map<std::string, double> &pose) {
  double score = 1.0;
  for (const EvaluationBasePtr &plugin : eval_plugins_) {
    score *= plugin->calculateScore(pose);
  }
  //    RCLCPP_INFO(LOGGER, "EVAL score = '%f'", score);

  return score;
}

}  // namespace plugins
}  // namespace reach

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(reach::plugins::MultiplicativeFactory,
                       reach::plugins::EvaluationBase)
