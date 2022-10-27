#pragma once

#include <reach_core/reach_database.h>

#include <map>
#include <set>
#include <vector>

namespace reach
{
class ComparisonResult
{
public:
  ComparisonResult(const std::set<std::string> db_names,
                   const std::map<std::string, std::size_t> reachability_mask_map);

  /** @brief Returns the IDs of the reach study targets that are reachable in all input reach studies */
  std::vector<std::string> getReachability(const std::vector<std::string>& dbs) const;

  /** @brief Returns the names of the studies in which the input reach study target was reachable */
  std::vector<std::string> getReachability(const std::string& target) const;

  /** @brief Returns a single string descriptor of the databases in which the input reach study target was reachable */
  std::string getReachabilityDescriptor(const std::string& target) const;

protected:
  /** @brief List of reach study database names */
  const std::set<std::string> db_names_;
  /** @brief Map of reach study target ID names to a bit mask corresponding to the indices of the reach studies in which
   * the target was reachable */
  const std::map<std::string, std::size_t> reachability_mask_map_;
  /** @brief Map of reachability bit masks to descriptive string names (e.g. 5 = 101 -> db0_AND_db1) */
  const std::map<std::size_t, std::string> mask_names_;
};

ComparisonResult compareDatabases(const std::vector<std::string>& db_files);
ComparisonResult compareDatabases(const std::vector<ReachDatabase>& dbs);

} // namespace reach
