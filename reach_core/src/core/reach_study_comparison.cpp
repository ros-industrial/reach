#include <reach_core/reach_study_comparison.h>
#include <reach_core/reach_database.h>

namespace reach
{
namespace core
{
static std::vector<std::string> getRecordIDs(const ReachDatabase& db)
{
  std::vector<std::string> keys;
  std::transform(db.begin(), db.end(), std::back_inserter(keys), [](const auto& pair) { return pair.first; });
  return keys;
}

ComparisonResult::ComparisonResult(const std::set<std::string> db_names_,
                                               const std::map<std::string, std::size_t> reachability_mask_map_)
  : db_names(std::move(db_names_)), reachability_mask_map(std::move(reachability_mask_map_))
{
}

std::vector<std::string> ComparisonResult::getReachability(const std::set<std::string>& names) const
{
  // Get the indices of the input study names in the internal member
  std::vector<std::size_t> name_idx;
  name_idx.reserve(names.size());
  for (const std::string& db : names)
  {
    auto it = std::find(db_names.begin(), db_names.end(), db);
    if (it == db_names.end())
      throw std::runtime_error("Database '" + db + "' is not known");

    name_idx.push_back(std::distance(db_names.begin(), it));
  }

  // Helper function to check if the mask is 1 at the given index
  auto check_mask = [](const std::size_t mask, const std::size_t idx) { return ((mask & (1 << idx)) >> idx) == 1; };

  // Loop over all record ids to find the ones that were reachable by all input database names
  std::vector<std::string> ids;
  for (auto it = reachability_mask_map.begin(); it != reachability_mask_map.end(); ++it)
  {
    bool all_reachable = std::all_of(name_idx.begin(), name_idx.end(),
                                     [&it, &check_mask](const std::size_t idx) { return check_mask(it->second, idx); });
    if (all_reachable)
      ids.push_back(it->first);
  }

  return ids;
}

std::map<std::size_t, std::string> ComparisonResult::createMaskNames() const
{
  // Compute the permuations of databases for comparison
  const std::size_t n_perm = pow(2, db_names.size());

  // Generate a list of all possible namespace permutations
  std::map<std::size_t, std::string> mask_name_map;
  mask_name_map[0] = "not_all";
  mask_name_map[n_perm - 1] = "all";

  for (std::size_t perm_ind = 1; perm_ind < n_perm - 1; ++perm_ind)
  {
    std::string ns_name = "";
    //    for (auto it = dbs.begin(); it != dbs.end(); ++it)
    for (std::size_t i = 0; i < db_names.size(); ++i)
    {
      const std::string db_name = *std::next(db_names.begin(), i);

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

ComparisonResult compareDatabases(const std::vector<std::string>& db_filenames)
{
  if (db_filenames.empty())
    throw std::runtime_error("Must provide at least one database for comparison");

  // Load databases to be compared
  std::map<std::string, ReachDatabase> dbs;
  for (const std::string& filename : db_filenames)
  {
    ReachDatabase db = reach::core::load(filename);
    dbs.emplace(db.getName(), std::move(db));
  }

  // Get the names of all the records in this database
  const std::vector<std::string> record_ids = getRecordIDs(dbs.begin()->second);

  // Check that all databases have the same records
  {
    for (const auto pair : dbs)
    {
      const std::vector<std::string> other_record_ids = getRecordIDs(pair.second);
      if (!std::equal(record_ids.begin(), record_ids.end(), other_record_ids.begin()))
        throw std::runtime_error("Database '" + pair.first +
                                 "' does not contain all the same records as this database");
    }
  }

  // Iterate over all records in the databases and compare whether or not they were reached in that database
  std::map<std::string, std::size_t> reachability_mask_map;
  for (const std::string& id : record_ids)
  {
    // Create a binary code based on whether the point was reached code LSB is msg.reach boolean of 1st database code <<
    // n is is msg.reach boolean of (n+1)th database
    std::size_t mask = 0;

    for (auto it = dbs.begin(); it != dbs.end(); ++it)
    {
      mask += static_cast<char>(it->second.get(id).reached) << std::distance(dbs.begin(), it);
    }

    reachability_mask_map[id] = mask;
  }

  // Get the database names for the output
  std::set<std::string> db_names;
  std::transform(dbs.begin(), dbs.end(), std::inserter(db_names, db_names.begin()), [](const auto& pair) { return pair.first; });

  return ComparisonResult(db_names, reachability_mask_map);
}

} // namespace core
} // namespace reach
