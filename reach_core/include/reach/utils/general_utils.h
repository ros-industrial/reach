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
void integerProgressPrinter(std::atomic<int>& current_counter,
                            std::atomic<int>& previous_pct,
                            const int total_size);

/**
 * @brief createFrame
 * @param pt
 * @param norm
 * @return
 */
Eigen::Isometry3d createFrame(const Eigen::Vector3f& pt,
                            const Eigen::Vector3f& norm);

} // namespace utils
} // namespace reach

#endif // REACH_UTILS_GENERAL_UTILS_H
