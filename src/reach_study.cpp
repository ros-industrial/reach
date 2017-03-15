#include <robot_reach_study/reach_study.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>
#include <atomic>

#include <robot_reach_study/reach_database.h>
#include <robot_reach_study/interactive_ik.h>
#include <robot_reach_study/ik_helper.h>
#include <robot_reach_study/SampleMesh.h>

//void loadDatabases(const std::vector<std::string>& names,
//                   const std::vector<std::string>& filenames,
//                   robot_reach_study::InteractiveIK& ik_visualizer)
//{
//  std::vector<std::pair<std::string, robot_reach_study::Database*>> data;
//  std::vector<robot_reach_study::Database> dbs(filenames.size());
//  std::vector<robot_reach_study::Database*> db_ptrs(filenames.size());

//  for(std::size_t i = 0; i < filenames.size(); i++)
//  {
//    if(!dbs[i].load(filenames[i]))
//    {
//      ROS_INFO("Cannot load %s", filenames[i].c_str());
//      continue;
//    }
//    db_ptrs[i] = &dbs[i];
//    std::pair<std::string, robot_reach_study::Database*> pair(names[i], db_ptrs[i]);
//    data.push_back(pair);
//  }

//  ik_visualizer.reachDiffVisualizer(data);

//}

Eigen::Affine3d createFrame(const Eigen::Vector3f& pt,
                            const Eigen::Vector3f& norm)
{
  // Initialize coordinate frame and set XYZ location
  Eigen::Affine3f p = Eigen::Affine3f::Identity();
  p.matrix()(0, 3) = pt(0);
  p.matrix()(1, 3) = pt(1);
  p.matrix()(2, 3) = pt(2);

  // Create plane from point normal
  Eigen::Hyperplane<float, 3> plane (norm, Eigen::Vector3f(0, 0, 0));

  // If the normal and global x-axis are not closely aligned
  if (std::abs(norm.dot(Eigen::Vector3f::UnitX()) < 0.90))
  {
    // Project the global x-axis onto the plane to generate the x-axis
    Eigen::Vector3f x_axis = plane.projection(Eigen::Vector3f::UnitX()).normalized();
    p.matrix().col(0).head<3>() = x_axis;
    p.matrix().col(1).head<3>() = norm.cross(x_axis);
    p.matrix().col(2).head<3>() = norm;
  }
  else
  {
    // Project the global y-axis onto the plane to generate the y-axis
    Eigen::Vector3f y_axis = plane.projection(Eigen::Vector3f::UnitY()).normalized();
    p.matrix().col(0).head<3>() = y_axis.cross(norm);
    p.matrix().col(1).head<3>() = y_axis;
    p.matrix().col(2).head<3>() = norm;
  }

  return p.cast<double>();
}

void integerProgressPrinter(std::atomic<int>& current_counter, std::atomic<int>& previous_pct, const size_t cloud_size)
{
  const float current_pct_float = (static_cast<float>(current_counter.load()) / static_cast<float>(cloud_size)) * 100.0;
  const int current_pct = static_cast<int>(current_pct_float);
  if(current_pct > previous_pct.load())
  {
    ROS_INFO("[%d%%]", current_pct);
  }
  previous_pct = current_pct;
}

float getAverageNeighborsCount(std::shared_ptr<robot_reach_study::Database>& db,
                               std::shared_ptr<robot_reach_study::IkHelper>& ik_helper)
{
  std::atomic<int> neighbor_count {0};
  ROS_INFO("------------------------------------------------");
  ROS_INFO("Beginning average neighbor count calculation");

  // Iterate
  #pragma parallel for
  for(int i = 0; i < db->count(); ++i)
  {
    boost::optional<robot_reach_study::ReachRecord> msg = db->get(std::to_string(i));
    if(msg && msg->reached)
    {
      std::vector<std::string> reached_pts;
      moveit::core::RobotState state(ik_helper->getCurrentRobotState());
      moveit::core::robotStateMsgToRobotState(msg->goal_state, state);
      ik_helper->reachNeighborsRecursive(db, msg->id, msg->goal, state, reached_pts);
      neighbor_count += static_cast<int>(reached_pts.size() - 1);
    }
    ROS_INFO("[%f]", (static_cast<float>(i) / static_cast<float>(db->count())) * 100.0);
  }

  float avg_neighbor_count = static_cast<float>(neighbor_count.load()) / static_cast<float>(db->count());

  ROS_INFO("------------------------------------------------");
  ROS_INFO("Average number of neighbors reached: %f", avg_neighbor_count);
  ROS_INFO("------------------------------------------------");

  return avg_neighbor_count;
}


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "point_cloud_IK_check");

  // ROS node handle
  ros::NodeHandle nh("~");

  // Get ROS parameters
  std::string desc;
  if(!nh.getParam("/robot_description", desc))
  {
    ROS_FATAL("Must set 'robot_description' parameter");
    return 0;
  }

  std::string pcd_file;
  if(!nh.getParam("pcd_file", pcd_file))
  {
    ROS_FATAL("Must set 'pcd_file' parameter");
    return 0;
  }

  std::string kin_group_name;
  if(!nh.getParam("move_group/kinematics", kin_group_name))
  {
    ROS_FATAL("Must set 'move_group/kinematics' parameter");
    return 0;
  }

  std::string manip_group_name;
  if(!nh.getParam("move_group/manipulability", manip_group_name))
  {
    ROS_FATAL("Must set 'move_group/manipulability' parameter");
    return 0;
  }

  std::string config_name;
  if(!nh.getParam("config_name", config_name))
  {
    ROS_FATAL("'config_name' parameter set incorrectly");
    return 0;
  }

  bool get_neighbors;
  if(!nh.getParam("get_avg_neighbor_count", get_neighbors))
  {
    ROS_FATAL("'get_avg_neighbor_count' parameter not set");
    return 0;
  }

  std::string world_frame;
  if(!nh.getParam("world_frame", world_frame))
  {
    ROS_FATAL("'world_frame' parameter must be set");
    return 0;
  }

  std::string object_frame;
  if(!nh.getParam("object_frame", object_frame))
  {
    ROS_FATAL("'object_frame' parameter must be set");
    return 0;
  }

  std::string object_mesh_filename;
  if(!nh.getParam("object_mesh_filename", object_mesh_filename))
  {
    ROS_FATAL("'object_mesh_filename' parameter must be set");
    return 0;
  }

  float mesh_sample_res;
  if(!nh.getParam("mesh_sample_res", mesh_sample_res))
  {
    ROS_FATAL("'mesh_sample_res' parameter must be set");
    return 0;
  }

  float cloud_output_res;
  if(!nh.getParam("cloud_output_res", cloud_output_res))
  {
    ROS_FATAL("'cloud_output_res' parameter must be set");
    return 0;
  }

  // Create a database where we will store the results of our ik search
  std::shared_ptr<robot_reach_study::Database> db (new robot_reach_study::Database ());

  // Initialize IK Helper and set parameters
  std::shared_ptr<robot_reach_study::IkHelper> helper ( new robot_reach_study::IkHelper ("/robot_description", kin_group_name, manip_group_name) );
  helper->setSolutionAttempts(1);
  helper->setSolutionTimeout(0.02);
  helper->setNeighborRadius(2.0 * cloud_output_res);

  // Visualize the reach database
  // has its own publishers which operate when ros::spin() is called
  robot_reach_study::InteractiveIK ik_visualizer (db, helper);

  // Do a service call to get reach object point cloud
  ros::ServiceClient client = nh.serviceClient<robot_reach_study::SampleMesh>("/sample_mesh");
  robot_reach_study::SampleMesh srv;
  srv.request.cloud_filename = pcd_file;
  srv.request.mesh_filename = object_mesh_filename;
  srv.request.world_name = world_frame;
  srv.request.object_name = object_frame;
  srv.request.sampling_resolution = mesh_sample_res;
  srv.request.output_resolution = cloud_output_res;
  client.waitForExistence(ros::Duration (5.0));
  if(client.call(srv))
  {
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/input_cloud", 1, true);
    srv.response.cloud.header.frame_id = world_frame;
    srv.response.cloud.header.stamp = ros::Time::now();
    pub.publish(srv.response.cloud);
  }
  else
  {
    ROS_FATAL("Failed to load reach object point cloud");
    return 0;
  }
  pcl::PointCloud<pcl::PointNormal> cloud;
  pcl::fromROSMsg(srv.response.cloud, cloud);

  // Add the reach object as a collision object in the planning scene
  std::vector<std::string> touch_links;
  touch_links.push_back(world_frame);
  if(!helper->addCollisionObjectToScene(object_mesh_filename, object_frame, touch_links))
  {
    return 0;
  }

  moveit_msgs::PlanningScene msg;
  helper->getPlanningScene()->getPlanningSceneMsg(msg);
  ik_visualizer.publishScene(msg);

  // Check if output data directory exists and create directory if it doesn't
  const std::string dir = ros::package::getPath("robot_reach_study") + "/output/" + config_name + "/";
  const char* char_dir = dir.c_str();

  if(!boost::filesystem::exists(char_dir))
  {
    boost::filesystem::path path(char_dir);
    boost::filesystem::create_directory(path);
  }

  // Attempt to load previously saved database and output point cloud
  const std::string opt_saved_db_name = dir + "optimized_reach.db";
  if(!db->load(opt_saved_db_name))
  {
    const std::string saved_db_name = dir + "reach.db";
    if(!db->load(saved_db_name))
    {
      ROS_INFO("------------------------------------------------");
      ROS_INFO("No reach database loaded");
      ROS_INFO("------------------------------------------------");

      // Rotation to flip the Z axis of the surface normal point
      const Eigen::AngleAxisd tool_z_rot(M_PI, Eigen::Vector3d::UnitY());

      // Loop through all points in point cloud and get IK solution
      std::atomic<int> current_counter;
      current_counter = 0;
      std::atomic<int> previous_pct;
      previous_pct = 0;

      #pragma omp parallel for
      for(unsigned int i = 0; i < cloud.points.size(); ++i)
      {
        // Get pose from point cloud array
        const pcl::PointNormal& pt = cloud.points[i];
        Eigen::Affine3d tgt_frame;
        tgt_frame = createFrame(pt.getArray3fMap(), pt.getNormalVector3fMap());
        tgt_frame = tgt_frame * tool_z_rot;
        geometry_msgs::Pose tgt_pose;
        tf::poseEigenToMsg(tgt_frame, tgt_pose);

        // Create robot state objects for goal and seed to be filled by ik_helper
        moveit::core::RobotState goal_state(helper->getCurrentRobotState());
        moveit::core::RobotState seed_state(helper->getCurrentRobotState());

        // Solve IK using setfromIK
        boost::optional<double> score = helper->solveIKFromSeed(tgt_pose, seed_state, goal_state);
        if(score)
        {
          auto msg = robot_reach_study::makeRecordSuccess(std::to_string(i), tgt_pose, seed_state, goal_state, *score);
          db->put(msg);
        }
        else
        {
          auto msg = robot_reach_study::makeRecordFailure(std::to_string(i), tgt_pose, seed_state, 0.0);
          db->put(msg);
        }

        // Print function progress
        current_counter++;
        integerProgressPrinter(current_counter, previous_pct, cloud.size());
      }

      // Save the results of the reach study to a database that we can query later
      const std::string reachdb_filename = dir + "reach.db";
      db->calculateResults();
      db->save(reachdb_filename);
    }
    else
    {
      ROS_INFO("------------------------------------------------");
      ROS_INFO("Reach database successfully loaded");
      ROS_INFO("------------------------------------------------");
      db->printResults();
    }

    // Optimize with nearest neighbors
    ROS_INFO("------------------------------------------------");
    ROS_INFO("Beginning optimization");

    // Create sequential vector to be randomized
    std::vector<int> rand_vec;
    for(int i = 0; i < db->count(); ++i)
    {
      rand_vec.push_back(i);
    }

    // Iterate
    std::atomic<int> current_counter;
    std::atomic<int> previous_pct;
    int n_opt = 0;
    float previous_score = 0.0;
    float pct_improve = 1.0;

    while(pct_improve > 0.01 && n_opt < 10)
    {
      ROS_INFO("Entering optimization loop %d", n_opt);
      previous_score = db->getNormalizedTotalScore();
      current_counter = 0;
      previous_pct = 0;

      // Randomize
      std::random_shuffle(rand_vec.begin(), rand_vec.end());

      #pragma parallel for
      for(std::size_t i = 0; i < rand_vec.size(); ++i)
      {
        robot_reach_study::ReachRecord msg = *db->get(std::to_string(rand_vec[i]));
        if(msg.reached)
        {
          std::vector<std::string> score = helper->reachNeighborsDirect(db, msg);
        }

        // Print function progress
        current_counter++;
        integerProgressPrinter(current_counter, previous_pct, cloud.size());
      }

      // Recalculate optimized reach study results
      db->calculateResults();
      pct_improve = std::abs((db->getNormalizedTotalScore() - previous_score)/previous_score);
      ++ n_opt;
    }

    // Save the optimized reach database
    const std::string opt_reachdb_filename = dir + "optimized_reach.db";
    db->calculateResults();
    db->save(opt_reachdb_filename);
    ROS_INFO("Optimization concluded");
    ROS_INFO("------------------------------------------------");

  }
  else
  {
    ROS_INFO("------------------------------------------------");
    ROS_INFO("Reach and optimized reach databases and point");
    ROS_INFO("cloud successfully loaded");
    ROS_INFO("------------------------------------------------");
    db->printResults();
  }

  if(get_neighbors)
  {
    const float n = getAverageNeighborsCount(db, helper);
    db->setAverageNeighborsCount(n);
    db->save(opt_saved_db_name);
  }

  // Load all databases
//  std::vector<std::string> db_filenames;
//  db_filenames.push_back(dir + "abb_optimized_reach.db");
//  db_filenames.push_back(dir + "abb2_optimized_reach.db");
//  db_filenames.push_back(dir + "abb3_optimized_reach.db");
//  db_filenames.push_back(dir + "motoman_optimized_reach.db");
//  db_filenames.push_back(dir + "motoman2_optimized_reach.db");
//  db_filenames.push_back(dir + "motoman3_optimized_reach.db");

//  std::vector<std::string> names;
//  names.push_back("abb");
//  names.push_back("abb2");
//  names.push_back("abb3");
//  names.push_back("motoman");
//  names.push_back("motoman2");
//  names.push_back("motoman3");

  //  loadDatabases(names, db_filenames, ik_visualizer);


  ik_visualizer.createReachMarkers();

  ros::spin();

  ros::shutdown();

  return 0;
}
