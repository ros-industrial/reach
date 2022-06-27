#pragma once

#include <reach_core/reach_database.h>

#include <map>
#include <set>
#include <vector>

namespace reach
{
struct ComparisonResult
{
  ComparisonResult(const std::set<std::string> db_names_, const std::map<std::string, std::size_t> reachability_mask_map_);

  const std::set<std::string> db_names;
  const std::map<std::string, std::size_t> reachability_mask_map;

  /**
     * @brief getReachability
     * @param dbs
     * @return
     */
  std::vector<std::string> getReachability(const std::set<std::string>& dbs) const;

  /**
     * @brief createMaskNames
     * @return
     */
  std::map<std::size_t, std::string> createMaskNames() const;
};

ComparisonResult compareDatabases(const std::vector<std::string>& db_files);

} // namespace reach
