#ifndef REACH_STUDY_H
#define REACH_STUDY_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <robot_reach_study/ik_helper.h>
#include <robot_reach_study/interactive_ik.h>
#include <sensor_msgs/PointCloud2.h>

namespace robot_reach_study
{
  /**
   * @brief The StudyParams struct contains all necessary parameters for the reach study
   */
  struct StudyParams
  {
    std::string config_name;              // reach study configuration name
    std::string fixed_frame;              // root, fixed frame of the URDF
    std::string object_frame;             // frame of the reach object in the URDF
    std::string object_mesh_filename;     // filename of the reach object mesh
    std::string pcd_file;                 // filename at which to save the point cloud representation of the reach object
    float n_neighbors;                    // number of nearest neighbors to use in normal estimation for the reach object
    float cloud_output_res;               // voxel size of the output reach object point cloud
    std::string kin_group_name;           // planning group used to solve for IK
    std::string manip_group_name;         // planning group used to calculate pose scoring
    bool get_neighbors;                   // flag for evaluating robot work area
    bool visualize_results;               // flag for publishing reach study data/markers for Rviz
    std::vector<std::string> compare_dbs; // list of database names with which to compare to the current database
    int cost_function;                    // enumeration defining the method used to score robot poses
    float distance_threshold;             // minimum distance from collision that the robot must be for the IK solution to be considered valid
  };

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
    ReachStudy(ros::NodeHandle& nh,
               StudyParams& sp);

    /**
     * @brief run
     * @return
     */
    bool run();

  private:

    void init();

    bool getReachObjectPointCloud();

    void runInitialReachStudy();

    void optimizeReachStudyResults();

    void getAverageNeighborsCount();

    bool compareDatabases();

    ros::NodeHandle nh_;
    std::shared_ptr<robot_reach_study::IkHelper> helper_;
    std::shared_ptr<robot_reach_study::InteractiveIK> ik_visualizer_;
    std::shared_ptr<robot_reach_study::Database> db_;
    StudyParams sp_;
    std::string dir_;
    std::string results_dir_;
    sensor_msgs::PointCloud2 cloud_msg_;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_;
  };

}

#endif // REACH_STUDY_H
