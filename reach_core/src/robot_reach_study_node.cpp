#include <ros/ros.h>
#include "reach/core/reach_study.h"
#include "reach/core/study_parameters.h"
#include <xmlrpcpp/XmlRpcException.h>

bool getStudyParameters(ros::NodeHandle& nh,
                        reach::core::StudyParameters& sp)
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

  if(!nh.getParam("results_directory", sp.results_directory))
  {
    ROS_WARN("'results_directory' parameter not set; using default file location");
    sp.results_directory = "";
  }

  if(!nh.getParam("object_frame", sp.object_frame))
  {
    ROS_ERROR("'object_frame' parameter must be set");
    return false;
  }

  if(!nh.getParam("reach_object/mesh_filename", sp.mesh_filename))
  {
    ROS_ERROR("Must set 'reach_object/mesh_filename' parameter");
    return false;
  }

  if(!nh.getParam("reach_object/pcd_filename", sp.pcd_filename))
  {
    ROS_ERROR("Must set 'reach_object/pcd_filename' parameter");
    return false;
  }

  if(!nh.getParam("reach_object/optimization_radius", sp.optimization_radius))
  {
    ROS_ERROR("Must set 'reach_object/optimization_radius' parameter");
    return false;
  }

  if(!nh.getParam("planning_group/kinematics", sp.kin_group_name))
  {
    ROS_ERROR("Must set 'planning_group/kinematics' parameter");
    return false;
  }

  if(!nh.getParam("planning_group/manipulability", sp.manip_group_name))
  {
    ROS_ERROR("Must set 'planning_group/manipulability' parameter");
    return false;
  }

  if(!nh.getParam("constraints/get_avg_neighbor_count", sp.get_neighbors))
  {
    ROS_ERROR("'constraints/get_avg_neighbor_count' parameter not set");
    return false;
  }

  if(!nh.getParam("constraints/distance_threshold", sp.distance_threshold))
  {
    ROS_ERROR("'constraints/distance_threshold' parameter must be set");
    return false;
  }

  if(!nh.getParam("constraints/compare_dbs", sp.compare_dbs))
  {
    ROS_WARN("'constraints/compare_dbs' parameter not set. No databases will be compared");
  }

  if(!nh.getParam("constraints/cost_function", sp.cost_function))
  {
    ROS_WARN("'constraints/cost_function' parameter not set. Defaulting to manipulability only cost function");
    sp.cost_function = 0;
  }

  if(!nh.getParam("/visualize_results", sp.visualize_results))
  {
    ROS_WARN("'/visualize_results' parameter not set; reach study results will not be visualized.");
    sp.visualize_results = false;
  }

  XmlRpc::XmlRpcValue seed_states;
  if(!nh.getParam("constraints/seed_states", seed_states))
  {
    ROS_WARN("'seed_states' parameter not set; using all zeros seed state");
  }
  else
  {
    sp.seed_states.resize(seed_states.size());
    try
    {
      for(int i = 0; i < seed_states.size(); ++i)
      {
        XmlRpc::XmlRpcValue& pose = seed_states[i];
        sp.seed_states[i].resize(pose.size());
        for(int j = 0; j < pose.size(); ++j)
        {
          sp.seed_states[i][j] = pose[j];
        }
      }
    }
    catch(XmlRpc::XmlRpcException& ex)
    {
      ROS_INFO_STREAM(ex.getMessage());
      return false;
    }
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_reach_study_node");
  ros::NodeHandle pnh("~"), nh;

  // Get the study parameters
  reach::core::StudyParameters sp;
  if(!getStudyParameters(pnh, sp))
  {
    return -1;
  }

  // Initialize the reach study
  reach::core::ReachStudy rs (nh, sp);

  // Run the reach study
  if(!rs.run(sp))
  {
    ROS_ERROR("Unable to perform the reach study");
    return -1;
  }

  return 0;
}
