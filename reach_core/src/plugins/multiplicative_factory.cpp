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
#include "multiplicative_factory.h"

#include <yaml-cpp/yaml.h>

namespace reach
{
const static std::string PACKAGE = "reach_core";
const static std::string PLUGIN_BASE_NAME = "reach::Evaluator";

MultiplicativeFactory::MultiplicativeFactory() : Evaluator(), class_loader_(PACKAGE, PLUGIN_BASE_NAME)
{
}

void MultiplicativeFactory::initialize(const YAML::Node& config)
{
  const YAML::Node plugin_configs = config["plugins"];

  eval_plugins_.reserve(plugin_configs.size());

  for (auto it = plugin_configs.begin(); it != plugin_configs.end(); ++it)
  {
    const YAML::Node& plugin_config = *it;
    Evaluator::Ptr plugin = class_loader_.createInstance(plugin_config["name"].as<std::string>());
    plugin->initialize(plugin_config);

    eval_plugins_.push_back(std::move(plugin));
  }

  if (eval_plugins_.empty())
    throw std::runtime_error("No valid plugins remain");
}

double MultiplicativeFactory::calculateScore(const std::map<std::string, double>& pose) const
{
  double score = 1.0;
  for (const Evaluator::Ptr& plugin : eval_plugins_)
  {
    score *= plugin->calculateScore(pose);
  }
  return score;
}

}  // namespace reach

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(reach::MultiplicativeFactory, reach::Evaluator)
