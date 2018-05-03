#include "reach/utils/general_utils.h"
#include <ros/console.h>

namespace reach
{
namespace utils
{

void integerProgressPrinter(std::atomic<int>& current_counter,
                            std::atomic<int>& previous_pct,
                            const int total_size)
{
  const float current_pct_float = (static_cast<float>(current_counter.load()) / static_cast<float>(total_size)) * 100.0;
  const int current_pct = static_cast<int>(current_pct_float);
  if(current_pct > previous_pct.load())
  {
    ROS_INFO("[%d%%]", current_pct);
  }
  previous_pct = current_pct;
}

} // namespace core
} // namespace reach

