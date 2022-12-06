#pragma once

#include <reach/types.h>

#include <map>
#include <vector>

namespace reach
{
class ComparisonResult
{
public:
  using mask = std::size_t;

  ComparisonResult(const std::vector<std::size_t> db_indices, const std::map<std::string, mask> reachability_mask_map);

  /** @brief Returns the IDs of the reach study targets that are reachable in all input reach studies */
  std::vector<std::string> getReachability(const std::vector<std::size_t>& dbs) const;

  /** @brief Returns the names of the studies in which the input reach study target was reachable */
  std::vector<std::size_t> getReachability(const std::string& target) const;

  /** @brief Returns a single string descriptor of the databases in which the input reach study target was reachable */
  std::string getReachabilityDescriptor(const std::string& target) const;

protected:
  /** @brief List of reach study database names */
  const std::vector<std::size_t> db_indices_;
  /** @brief Map of reach study target ID names to a bit mask corresponding to the indices of the reach studies in which
   * the target was reachable */
  const std::map<std::string, mask> reachability_mask_map_;
  /** @brief Map of reachability bit masks to descriptive string names (e.g. 5 = 101 -> db0_AND_db1) */
  const std::map<mask, std::string> mask_names_;
};

ComparisonResult compare(const std::vector<std::string>& db_files);
ComparisonResult compare(const std::vector<ReachResult>& results);

}  // namespace reach
