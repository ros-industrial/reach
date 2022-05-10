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
#ifndef MOVEIT_REACH_PLUGINS_EVALUATION_MANIPULABILITY_EVALUATION_H
#define MOVEIT_REACH_PLUGINS_EVALUATION_MANIPULABILITY_EVALUATION_H

#include <reach_core/plugins/evaluation_base.h>

namespace moveit_reach_plugins {
namespace {
const rclcpp::Logger LOGGER =
    rclcpp::get_logger("moveit_reach_plugins.ManipulabilityMoveIt");
}
namespace evaluation {

class ManipulabilityMoveIt : public reach::plugins::EvaluationBase {
 public:
  ManipulabilityMoveIt();
  ~ManipulabilityMoveIt() {
    model_.reset();
    //    delete jmg_;
  }

  virtual bool initialize(
      std::string& name, rclcpp::Node::SharedPtr node,
      const std::shared_ptr<const moveit::core::RobotModel> model) override;

  virtual double calculateScore(
      const std::map<std::string, double>& pose) override;

 private:
  moveit::core::RobotModelConstPtr model_;

  const moveit::core::JointModelGroup* jmg_;
};

}  // namespace evaluation
}  // namespace moveit_reach_plugins

#endif  // MOVEIT_REACH_PLUGINS_EVALUATION_MANIPULABILITY_EVALUATION_H
