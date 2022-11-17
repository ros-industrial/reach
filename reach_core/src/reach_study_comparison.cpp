#include <reach_core/reach_study_comparison.h>
#include <reach_core/reach_database.h>

namespace reach
{
static std::vector<std::string> getRecordIDs(const ReachDatabase& db)
{
  std::vector<std::string> keys;
  std::transform(db.begin(), db.end(), std::back_inserter(keys), [](const auto& pair) { return pair.first; });
  return keys;
}

static std::map<std::size_t, std::string> createMaskNames(const std::set<std::string>& db_names)
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

/* @brief Helper function to check if the mask is 1 at the given index */
static bool checkMask(const std::size_t mask, const std::size_t idx)
{
  return ((mask & (1 << idx)) >> idx) == 1;
}

ComparisonResult::ComparisonResult(const std::set<std::string> db_names,
                                   const std::map<std::string, std::size_t> reachability_mask_map)
  : db_names_(std::move(db_names))
  , reachability_mask_map_(std::move(reachability_mask_map))
  , mask_names_(createMaskNames(db_names_))
{
}

std::vector<std::string> ComparisonResult::getReachability(const std::vector<std::string>& dbs) const
{
  // Convert to a set
  std::set<std::string> db_set;
  std::copy(dbs.begin(), dbs.end(), std::inserter(db_set, db_set.begin()));

  // Get the indices of the input study names in the internal member
  std::vector<std::size_t> db_idx;
  db_idx.reserve(db_set.size());
  for (const std::string& db : db_set)
  {
    auto it = std::find(db_names_.begin(), db_names_.end(), db);
    if (it == db_names_.end())
      throw std::runtime_error("Database '" + db + "' is not known");

    db_idx.push_back(std::distance(db_names_.begin(), it));
  }

  // Loop over all record ids to find the ones that were reachable by all input database names
  std::vector<std::string> ids;
  for (auto it = reachability_mask_map_.begin(); it != reachability_mask_map_.end(); ++it)
  {
    bool all_reachable =
        std::all_of(db_idx.begin(), db_idx.end(), [&it](const std::size_t idx) { return checkMask(it->second, idx); });
    if (all_reachable)
      ids.push_back(it->first);
  }

  return ids;
}

std::vector<std::string> ComparisonResult::getReachability(const std::string& target) const
{
  const std::size_t mask = reachability_mask_map_.at(target);
  std::vector<std::string> names;
  names.reserve(db_names_.size());
  for (auto it = db_names_.begin(); it != db_names_.end(); ++it)
  {
    auto idx = std::distance(db_names_.begin(), it);
    if (checkMask(mask, idx))
      names.push_back(*it);
  }
  return names;
}

std::string ComparisonResult::getReachabilityDescriptor(const std::string& target) const
{
  return mask_names_.at(reachability_mask_map_.at(target));
}

ComparisonResult compareDatabases(const std::vector<std::string>& db_filenames)
{
  // Load databases to be compared
  std::vector<ReachDatabase> dbs;
  dbs.reserve(db_filenames.size());
  for (const std::string& filename : db_filenames)
  {
    ReachDatabase db = reach::load(filename);
    dbs.push_back(std::move(db));
  }

  return compareDatabases(dbs);
}

ComparisonResult compareDatabases(const std::vector<ReachDatabase>& dbs)
{
  if (dbs.empty())
    throw std::runtime_error("Must provide at least one database for comparison");

  // Get the names of all the records in this database
  const std::vector<std::string> record_ids = getRecordIDs(*dbs.begin());

  // Check that all databases have the same records
  {
    for (const auto db : dbs)
    {
      const std::vector<std::string> other_record_ids = getRecordIDs(db);
      if (!std::equal(record_ids.begin(), record_ids.end(), other_record_ids.begin()))
        throw std::runtime_error("Database '" + db.name + "' does not contain all the same records as this database");
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
      mask += static_cast<char>(it->get(id).reached) << std::distance(dbs.begin(), it);
    }

    reachability_mask_map[id] = mask;
  }

  // Get the database names for the output
  std::set<std::string> db_names;
  std::transform(dbs.begin(), dbs.end(), std::inserter(db_names, db_names.begin()),
                 [](const ReachDatabase& db) { return db.name; });

  return ComparisonResult(db_names, reachability_mask_map);
}

}  // namespace reach
