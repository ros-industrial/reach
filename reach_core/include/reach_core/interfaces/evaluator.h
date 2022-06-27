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
#ifndef REACH_CORE_PLUGINS_EVALUATION_EVALUATION_BASE
#define REACH_CORE_PLUGINS_EVALUATION_EVALUATION_BASE

#include <boost/shared_ptr.hpp>
#include <vector>
#include <xmlrpcpp/XmlRpcValue.h>

namespace reach
{
/**
 * @brief The Evaluator class
 */
class Evaluator
{
public:
  using Ptr = boost::shared_ptr<Evaluator>;
  using ConstPtr = boost::shared_ptr<const Evaluator>;

  Evaluator() = default;
  virtual ~Evaluator() = default;

  /**
   * @brief initialize
   * @param config
   */
  virtual void initialize(XmlRpc::XmlRpcValue& config) = 0;

  /**
   * @brief calculateScore
   * @param pose
   * @return
   */
  virtual double calculateScore(const std::map<std::string, double>& pose) const = 0;
};

}  // namespace reach

#endif  // REACH_CORE_PLUGINS_EVALUATION_EVALUATION_BASE
