#ifndef REACH_CORE_STUDY_VISUALIZER_H
#define REACH_CORE_STUDY_VISUALIZER_H

#include <reach/core/reach_database.h>
#include <reach/plugins/reach_display_base.h>
#include <reach/plugins/ik_solver_base.h>
#include <reach/core/ik_helper.h>

namespace reach
{
namespace core
{

/**
 * @brief The ReachVisualizer class displays the results of the reach study and provides an interface for visualizing the robot's work
 * area from a given pose, and recalculating the IK solution at a given target pose
 */
class ReachVisualizer
{
public:

  /**
   * @brief ReachVisualizer
   * @param db
   * @param solver
   * @param display
   * @param neighbor_radius
   * @param search_tree
   */
  ReachVisualizer(ReachDatabasePtr db,
                  reach::plugins::IKSolverBasePtr solver,
                  reach::plugins::DisplayBasePtr display,
                  const double neighbor_radius,
                  SearchTreePtr search_tree = nullptr);

  void update();

private:

  void reSolveIKCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb);

  void showResultCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb);

  void showSeedCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb);

  void reachNeighborsDirectCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb);

  void reachNeighborsRecursiveCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &fb);

  ReachDatabasePtr db_;

  reach::plugins::IKSolverBasePtr solver_;

  reach::plugins::DisplayBasePtr display_;

  SearchTreePtr search_tree_;

  double neighbor_radius_;
};
typedef std::shared_ptr<ReachVisualizer> ReachVisualizerPtr;

} // namespace core
} // namespace reach

#endif // REACH_CORE_STUDY_VISUALIZER_H
