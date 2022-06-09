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
#ifndef REACH_UTILS_GENERAL_UTILS_H
#define REACH_UTILS_GENERAL_UTILS_H

#include <atomic>
#include <Eigen/Dense>

namespace reach
{
namespace utils
{
/**
 * @brief integerProgressPrinter
 * @param current_counter
 * @param previous_pct
 * @param total_size
 */
void integerProgressPrinter(std::atomic<int>& current_counter, std::atomic<int>& previous_pct, const int total_size);

/**
 * @brief createFrame
 * @param pt
 * @param norm
 * @return
 */
Eigen::Isometry3d createFrame(const Eigen::Vector3f& pt, const Eigen::Vector3f& norm);

}  // namespace utils
}  // namespace reach

#endif  // REACH_UTILS_GENERAL_UTILS_H
