#ifndef REACH_CORE_REACH_STUDY_H
#define REACH_CORE_REACH_STUDY_H

#include <reach/core/study_parameters.h>
#include <reach/core/ik_helper.h>
#include <reach/core/reach_visualizer.h>
#include <reach/plugins/ik_solver_base.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_loader.h>
#include <sensor_msgs/PointCloud2.h>

namespace moveit
{
namespace core
{
class RobotModel;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;
}
}

namespace reach
{
namespace core
{

/**
 * @brief The ReachStudy class
 */
class ReachStudy
{
public:

  /**
   * @brief ReachStudy
   * @param nh
   */
  ReachStudy(const ros::NodeHandle& nh);

  /**
   * @brief run
   * @param sp
   * @return
   */
  bool run(const StudyParameters& sp);

private:

  bool initializeStudy();

  bool getReachObjectPointCloud();

  void runInitialReachStudy();

  void optimizeReachStudyResults();

  void getAverageNeighborsCount();

  bool compareDatabases();

  ros::NodeHandle nh_;
  
  StudyParameters sp_;
  
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_;
  
  ReachDatabasePtr db_;

  // Plugins
  pluginlib::ClassLoader<reach::plugins::IKSolverBase> solver_loader_;
  pluginlib::ClassLoader<reach::plugins::DisplayBase> display_loader_;
  reach::plugins::IKSolverBasePtr ik_solver_;
  reach::plugins::DisplayBasePtr display_;
  
  ReachVisualizerPtr visualizer_;

  SearchTreePtr search_tree_;

  moveit::core::RobotModelConstPtr model_;
  
  std::string dir_;
  
  std::string results_dir_;
  
  sensor_msgs::PointCloud2 cloud_msg_;
};

} // namespace core
} // namespace reach

#endif // REACH_CORE_REACH_STUDY_H
