#ifndef REACH_CORE_IK_HELPER_H
#define REACH_CORE_IK_HELPER_H

#include <reach/core/reach_database.h>
#include <reach/core/study_parameters.h>
#include <reach/plugins/ik_solver_base.h>

#include <boost/optional.hpp>
#include <flann/flann.h>
#include <flann/algorithms/kdtree_single_index.h>

namespace reach
{
namespace core
{

struct NeighborReachResult
{
  std::vector<std::string> reached_pts;
  double joint_distance = 0;
};

typedef flann::KDTreeSingleIndex<flann::L2_3D<double>> SearchTree;
typedef std::shared_ptr<SearchTree> SearchTreePtr;

NeighborReachResult reachNeighborsDirect(std::shared_ptr<ReachDatabase> db,
                                         const reach_msgs::ReachRecord& rec,
                                         reach::plugins::IKSolverBasePtr solver,
                                         const double radius,
                                         SearchTreePtr search_tree = nullptr);

void reachNeighborsRecursive(std::shared_ptr<ReachDatabase> db,
                             const reach_msgs::ReachRecord& msg,
                             reach::plugins::IKSolverBasePtr solver,
                             const double radius,
                             NeighborReachResult result,
                             SearchTreePtr search_tree = nullptr);

} // namespace core
} // namespace reach

#endif // REACH_CORE_IK_HELPER_H
