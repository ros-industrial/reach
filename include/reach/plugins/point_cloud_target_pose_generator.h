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
#ifndef REACH_PLUGINS_POINT_CLOUD_TARGET_POSE_GENERATOR_H
#define REACH_PLUGINS_POINT_CLOUD_TARGET_POSE_GENERATOR_H

#include <reach/interfaces/target_pose_generator.h>

namespace reach
{
class PointCloudTargetPoseGenerator : public TargetPoseGenerator
{
public:
  PointCloudTargetPoseGenerator(std::string filename);

  VectorIsometry3d generate() const override;

private:
  std::string filename_;
};

struct PointCloudTargetPoseGeneratorFactory : public TargetPoseGeneratorFactory
{
  using TargetPoseGeneratorFactory::TargetPoseGeneratorFactory;
  TargetPoseGenerator::ConstPtr create(const YAML::Node& config) const override;
};

}  // namespace reach

#endif  // REACH_PLUGINS_POINT_CLOUD_TARGET_POSE_GENERATOR_H
