#ifndef INTERACTIVE_IK_H
#define INTERACTIVE_IK_H

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <robot_reach_study/reach_database.h>
#include <robot_reach_study/ik_helper.h>

namespace robot_reach_study
{

/**
 * @brief The InteractiveIK class displays the results of the reach study and provides an interface for visualizing the robot's work
 * area from a given pose, recalculating the IK solution at a given target pose, and comparing reach databases.
 */
class InteractiveIK
{
public:

  /**
   * @brief Constructor for InteractiveIK class
   * @param db
   * @param ik_helper
   */
  InteractiveIK(ros::NodeHandle& nh,
                std::shared_ptr<robot_reach_study::Database>& db,
                std::shared_ptr<robot_reach_study::IkHelper>& ik_helper);

  /**
   * @brief createReachMarkers creates an Interactive Marker for each pose in the input reach study database.
   */
  void createReachMarkers();

  /**
   * @brief publishScene publishes the planning scene and any added collision objects
   * @param msg
   */
  void publishScene(const moveit_msgs::PlanningScene& msg);

  /**
   * @brief reachDiffVisualizer compares the results of the current reach study database with those in the input vector.
   * Any subset of reach studies can be compared to visualize which configuration did or did not find a solution for the shared
   * set of poses.
   *
   * @param data a vector of pairs, the first element of which is the configuration name of the reach study and the second of which
   * is the associated reach study database.
   */
  void reachDiffVisualizer(std::vector<std::pair<std::string, std::shared_ptr<robot_reach_study::Database>>> data);

  /**
   * @brief setMarkerFrame
   * @param frame
   */
  void setMarkerFrame(const std::string& frame) {fixed_frame_ = frame;}

  /**
   * @brief setMarkerScale
   * @param scale
   */
  void setMarkerScale(const double scale) {marker_scale_ = scale;}

private:

  void addRecord(const robot_reach_study::ReachRecord& rec);

  void publishMarkerArray(std::vector<std::string>& msg_ids);

  void reSolveIKCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb);

  void showResultCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb);

  void showSeedCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb);

  void reachNeighborsDirectCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb);

  void reachNeighborsRecursiveCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &fb);

  interactive_markers::InteractiveMarkerServer server_;
  std::shared_ptr<robot_reach_study::Database> db_;
  interactive_markers::MenuHandler menu_handler_;
  ros::NodeHandle nh_;
  ros::Publisher state_pub_;
  ros::Publisher scene_pub_;
  ros::Publisher neighbor_pub_;
  ros::Publisher diff_pub_;
  std::shared_ptr<robot_reach_study::IkHelper> ik_helper_;
  std::string fixed_frame_ = "world";
  double marker_scale_ = 0.150;
};

}

#endif // INTERACTIVE_IK_H
