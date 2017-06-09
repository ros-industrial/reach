#include <ros/ros.h>
#include <robot_reach_study/reach_study.h>

bool getStudyParameters(ros::NodeHandle& nh,
                        robot_reach_study::StudyParams& sp)
{
  if(!nh.getParam("config_name", sp.config_name))
  {
    ROS_ERROR("'config_name' parameter set incorrectly");
    return false;
  }

  if(!nh.getParam("fixed_frame", sp.fixed_frame))
  {
    ROS_ERROR("'fixed_frame' parameter must be set");
    return false;
  }

  if(!nh.getParam("object_frame", sp.object_frame))
  {
    ROS_ERROR("'object_frame' parameter must be set");
    return false;
  }

  if(!nh.getParam("reach_object/pcd_file", sp.pcd_file))
  {
    ROS_ERROR("Must set 'pcd_file' parameter");
    return false;
  }

  if(!nh.getParam("reach_object/object_mesh_filename", sp.object_mesh_filename))
  {
    ROS_ERROR("'object_mesh_filename' parameter must be set");
    return false;
  }

  if(!nh.getParam("reach_object/n_neighbors", sp.n_neighbors))
  {
    ROS_ERROR("'n_neighbors' parameter must be set");
    return false;
  }

  if(!nh.getParam("reach_object/cloud_output_res", sp.cloud_output_res))
  {
    ROS_ERROR("'cloud_output_res' parameter must be set");
    return false;
  }

  if(!nh.getParam("move_group/kinematics", sp.kin_group_name))
  {
    ROS_ERROR("Must set 'move_group/kinematics' parameter");
    return false;
  }

  if(!nh.getParam("move_group/manipulability", sp.manip_group_name))
  {
    ROS_ERROR("Must set 'move_group/manipulability' parameter");
    return false;
  }

  if(!nh.getParam("constraints/get_avg_neighbor_count", sp.get_neighbors))
  {
    ROS_ERROR("'get_avg_neighbor_count' parameter not set");
    return false;
  }

  if(!nh.getParam("constraints/distance_threshold", sp.distance_threshold))
  {
    ROS_ERROR("'distance_threshold' parameter must be set");
    return false;
  }

  if(!nh.getParam("constraints/compare_dbs", sp.compare_dbs))
  {
    ROS_ERROR("'compare_dbs' parameter not set. No databases will be compared");
  }

  if(!nh.getParam("constraints/cost_function", sp.cost_function))
  {
    ROS_ERROR("'cost_function' parameter not set. Defaulting to manipulability only cost function");
    sp.cost_function = 0;
  }

  if(!nh.getParam("/visualize_results", sp.visualize_results))
  {
    ROS_ERROR("'visualize_results' parameter not set; reach study results will not be visualized.");
    sp.visualize_results = false;
  }

  return true;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_reach_study_node");
  ros::NodeHandle pnh("~"), nh;

  // Get the study parameters
  robot_reach_study::StudyParams sp;
  if(!getStudyParameters(pnh, sp))
  {
    return -1;
  }

  // Initialize the reach study
  robot_reach_study::ReachStudy rs (nh, sp);

  // Run the reach study
  if(!rs.run())
  {
    ROS_ERROR("Unable to perform the reach study");
    return -1;
  }

  return 0;
}
