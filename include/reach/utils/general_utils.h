#ifndef REACH_UTILS_GENERAL_UTILS_H
#define REACH_UTILS_GENERAL_UTILS_H

#include <atomic>

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
inline void integerProgressPrinter(std::atomic<int>& current_counter,
                                   std::atomic<int>& previous_pct,
                                   const int total_size);

} // namespace utils
} // namespace reach

#endif // REACH_UTILS_GENERAL_UTILS_H
