#ifndef INTERACTIVE_IK_H
#define INTERACTIVE_IK_H

#include <interactive_markers/interactive_marker_server.h>
#include <robot_reach_study/reach_database.h>
#include <robot_reach_study/ik_helper.h>
#include <interactive_markers/menu_handler.h>
#include <eigen_conversions/eigen_msg.h>


namespace robot_reach_study
{

// Interactive marker server impl for our reach study results
/*
 * Menu entries:
 * 1. Show result
 * 2. Show seed
 * 3. Recompute
 */

class InteractiveIK
{
public:
  InteractiveIK(std::shared_ptr<robot_reach_study::Database>& db,
                std::shared_ptr<robot_reach_study::IkHelper>& ik_helper);

  void createReachMarkers();
  void publishScene(const moveit_msgs::PlanningScene& msg);
  void reachDiffVisualizer(std::vector<std::pair<std::string, std::shared_ptr<robot_reach_study::Database>>> data);

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
  interactive_markers::MenuHandler menu_handler;
  ros::Publisher state_pub_;
  ros::Publisher scene_pub_;
  ros::Publisher neighbor_pub_;
  ros::Publisher diff_pub_;
  std::shared_ptr<robot_reach_study::IkHelper> ik_helper_;
};

}

#endif // INTERACTIVE_IK_H
