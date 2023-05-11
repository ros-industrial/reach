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
#include <reach/plugins/multiplicative_evaluator.h>
#include <reach/plugin_utils.h>

#include <boost/algorithm/string.hpp>
#include <yaml-cpp/yaml.h>

namespace reach
{
MultiplicativeEvaluator::MultiplicativeEvaluator(std::vector<Evaluator::ConstPtr> evaluators)
  : evaluators_(std::move(evaluators))
{
}

double MultiplicativeEvaluator::calculateScore(const std::map<std::string, double>& pose) const
{
  double score = 1.0;
  for (const Evaluator::ConstPtr& eval : evaluators_)
  {
    score *= eval->calculateScore(pose);
  }
  return score;
}

MultiplicativeEvaluatorFactory::MultiplicativeEvaluatorFactory()
{
  loader_.search_libraries_env = SEARCH_LIBRARIES_ENV;
  boost::split(loader_.search_libraries, PLUGIN_LIBRARIES, boost::is_any_of(":"), boost::token_compress_on);
  loader_.search_system_folders = true;
}

Evaluator::ConstPtr MultiplicativeEvaluatorFactory::create(const YAML::Node& config) const
{
  const YAML::Node plugin_configs = config["plugins"];
  std::vector<Evaluator::ConstPtr> evaluators;
  evaluators.reserve(plugin_configs.size());

  for (auto it = plugin_configs.begin(); it != plugin_configs.end(); ++it)
  {
    const YAML::Node& plugin_config = *it;
    EvaluatorFactory::Ptr factory = loader_.createInstance<EvaluatorFactory>(get<std::string>(plugin_config, "name"));
    evaluators.push_back(factory->create(plugin_config));
  }

  if (evaluators.empty())
    throw std::runtime_error("No valid plugins remain");

  return std::make_shared<MultiplicativeEvaluator>(evaluators);
}

}  // namespace reach
