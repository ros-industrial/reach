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
#ifndef reach_INTERFACES_EVALUATOR_H
#define reach_INTERFACES_EVALUATOR_H

#include <string>
#include <map>
#include <memory>
#include <vector>

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
}
}  // namespace boost
#endif

namespace reach
{
/**
 * @brief Interface for evaluating the "fitness" of a robot joint pose (i.e., IK solution)
 */
struct Evaluator
{
  using Ptr = std::shared_ptr<Evaluator>;
  using ConstPtr = std::shared_ptr<const Evaluator>;

  Evaluator() = default;
  virtual ~Evaluator() = default;

  /**
   * @brief Calculates a score representing the "fitness" (i.e., quality of reachability) for a given robot pose.
   * @details The better the reachability of the pose, the higher the score should be.
   */
  virtual double calculateScore(const std::map<std::string, double>& pose) const = 0;

#ifdef BUILD_PYTHON
  double calculateScore(const boost::python::dict& pose) const;
#endif
};

/**
 * @brief Plugin interface for generating evaluator interfaces
 */
struct EvaluatorFactory
{
  using Ptr = std::shared_ptr<EvaluatorFactory>;
  using ConstPtr = std::shared_ptr<const EvaluatorFactory>;

  EvaluatorFactory() = default;
  virtual ~EvaluatorFactory() = default;

  virtual Evaluator::ConstPtr create(const YAML::Node& config) const = 0;

  static std::string getSection()
  {
    return EVALUATOR_SECTION;
  }

#ifdef BUILD_PYTHON
  Evaluator::ConstPtr create(const boost::python::dict& pyyaml_config) const;
#endif
};

}  // namespace reach

#endif  // reach_INTERFACES_EVALUATOR_H
