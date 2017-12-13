#ifndef REACH_CORE_REACH_STUDY_H
#define REACH_CORE_REACH_STUDY_H

#include "reach/core/study_parameters.h"
#include "reach/core/ik_helper.h"
#include "reach/core/reach_visualizer.h"

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

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
   * @param sp
   */
  ReachStudy(ros::NodeHandle& nh);

  /**
   * @brief run
   * @return
   */
  bool run(const StudyParameters& sp);

private:

  void initializeStudy(const StudyParameters& sp);

  bool getReachObjectPointCloud();

  void runInitialReachStudy();

  void optimizeReachStudyResults();

  void getAverageNeighborsCount();

  bool compareDatabases();

  ros::NodeHandle nh_;
  
  std::shared_ptr<IkHelper> helper_;
  
  std::shared_ptr<ReachVisualizer> visualizer_;
  
  std::shared_ptr<ReachDatabase> db_;
  
//  StudyParameters sp_;
  
  std::string dir_;
  
  std::string results_dir_;
  
  sensor_msgs::PointCloud2 cloud_msg_;
  
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_;
};

} // namespace core
} // namespace reach

#endif // REACH_CORE_REACH_STUDY_H
