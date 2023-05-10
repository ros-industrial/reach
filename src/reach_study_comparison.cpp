#include <reach/reach_study_comparison.h>
#include <reach/types.h>

#include <numeric>

namespace reach
{
static std::map<std::size_t, std::string> createMaskNames(const std::vector<std::size_t>& db_names)
{
  // Compute the permuations of databases for comparison
  const std::size_t n_perm = pow(2, db_names.size());

  // Generate a list of all possible namespace permutations
  std::map<std::size_t, std::string> mask_name_map;
  mask_name_map[0] = "none";
  mask_name_map[n_perm - 1] = "all";

  for (std::size_t perm_ind = 1; perm_ind < n_perm - 1; ++perm_ind)
  {
    std::string ns_name = "";
    for (std::size_t i = 0; i < db_names.size(); ++i)
    {
      const std::string db_name = std::to_string(*std::next(db_names.begin(), i));

      if (((perm_ind >> i) & 1) == 1)
      {
        if (ns_name == "")
        {
          ns_name += db_name;
        }
        else
        {
          ns_name += "_AND_" + db_name;
        }
      }
    }
    mask_name_map[perm_ind] = ns_name;
  }

  return mask_name_map;
}

/* @brief Helper function to check if the mask is 1 at the given index */
static bool checkMask(const std::size_t mask, const std::size_t idx)
{
  return ((mask & (1 << idx)) >> idx) == 1;
}

ComparisonResult::ComparisonResult(const std::vector<std::size_t> db_indices,
                                   const std::map<std::string, std::size_t> reachability_mask_map)
  : db_indices_(std::move(db_indices))
  , reachability_mask_map_(std::move(reachability_mask_map))
  , mask_names_(createMaskNames(db_indices_))
{
}

std::vector<std::string> ComparisonResult::getReachability(const std::vector<std::size_t>& dbs) const
{
  // Loop over all record ids to find the ones that were reachable by all input database names
  std::vector<std::string> ids;
  for (auto it = reachability_mask_map_.begin(); it != reachability_mask_map_.end(); ++it)
  {
    bool all_reachable = std::all_of(db_indices_.begin(), db_indices_.end(),
                                     [&it](const std::size_t idx) { return checkMask(it->second, idx); });
    if (all_reachable)
      ids.push_back(it->first);
  }

  return ids;
}

std::vector<std::size_t> ComparisonResult::getReachability(const std::string& target) const
{
  const std::size_t mask = reachability_mask_map_.at(target);
  std::vector<std::size_t> db_indices;
  db_indices.reserve(db_indices_.size());
  for (auto it = db_indices_.begin(); it != db_indices_.end(); ++it)
  {
    auto idx = std::distance(db_indices_.begin(), it);
    if (checkMask(mask, idx))
      db_indices.push_back(*it);
  }
  return db_indices;
}

std::string ComparisonResult::getReachabilityDescriptor(const std::string& target) const
{
  return mask_names_.at(reachability_mask_map_.at(target));
}

ComparisonResult compare(const std::vector<std::string>& db_filenames)
{
  // Load databases to be compared
  std::vector<ReachResult> results;
  results.reserve(db_filenames.size());
  for (const std::string& filename : db_filenames)
  {
    ReachDatabase db = reach::load(filename);
    results.push_back(db.results.back());
  }

  return compare(results);
}

ComparisonResult compare(const std::vector<ReachResult>& results)
{
  if (results.empty())
    throw std::runtime_error("Must provide at least one database for comparison");

  const std::size_t n_records = results.front().size();

  // Get the indices of all the records in this database
  std::vector<std::size_t> record_idxs(n_records);
  std::iota(record_idxs.begin(), record_idxs.end(), 0);

  // Check that all databases have the same records
  std::all_of(results.begin(), results.end(), [&n_records](const ReachResult& db) { return db.size() == n_records; });

  // Iterate over all records in the databases and compare whether or not they were reached in that database
  std::map<std::string, ComparisonResult::mask> reachability_mask_map;
  for (const std::size_t& idx : record_idxs)
  {
    // Create a binary code based on whether the point was reached code LSB is msg.reach boolean of 1st database code <<
    // n is is msg.reach boolean of (n+1)th database
    ComparisonResult::mask mask = 0;

    for (auto it = results.begin(); it != results.end(); ++it)
    {
      mask += static_cast<char>(it->at(idx).reached) << std::distance(results.begin(), it);
    }

    reachability_mask_map[std::to_string(idx)] = mask;
  }

  // Get the database names for the output
  std::vector<std::size_t> db_indices(results.size());
  std::iota(db_indices.begin(), db_indices.end(), 0);
  return ComparisonResult(db_indices, reachability_mask_map);
}

}  // namespace reach
