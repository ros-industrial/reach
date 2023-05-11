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
#ifndef REACH_PLUGINS_MULTIPLICATIVE_EVALUATOR_H
#define REACH_PLUGINS_MULTIPLICATIVE_EVALUATOR_H

#include <reach/interfaces/evaluator.h>

#include <boost_plugin_loader/plugin_loader.hpp>

namespace reach
{
class MultiplicativeEvaluator : public Evaluator
{
public:
  MultiplicativeEvaluator(std::vector<Evaluator::ConstPtr> evaluators);

  double calculateScore(const std::map<std::string, double>& pose) const override;

private:
  std::vector<Evaluator::ConstPtr> evaluators_;
};

class MultiplicativeEvaluatorFactory : public EvaluatorFactory
{
public:
  MultiplicativeEvaluatorFactory();

  Evaluator::ConstPtr create(const YAML::Node& config) const override;

private:
  mutable boost_plugin_loader::PluginLoader loader_;
};

}  // namespace reach

#endif  // REACH_PLUGINS_MULTIPLICATIVE_EVALUATOR_H
