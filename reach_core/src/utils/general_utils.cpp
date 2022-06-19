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
#include <reach_core/utils/general_utils.h>

#include <iostream>

namespace reach
{
namespace utils
{
void integerProgressPrinter(std::atomic<int>& current_counter, std::atomic<int>& previous_pct, const int total_size)
{
  const float current_pct_float = (static_cast<float>(current_counter.load()) / static_cast<float>(total_size)) * 100.0;
  const int current_pct = static_cast<int>(current_pct_float);
  if (current_pct > previous_pct.load())
  {
    std::cout << "[" << current_pct << "%]" << std::endl;
  }
  previous_pct = current_pct;
}

}  // namespace utils
}  // namespace reach
