#include <atomic>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/point_cloud.h>
#include <robot_reach_study/reach_study.h>
#include <robot_reach_study/SampleMesh.h>
#include <robot_reach_study/utils.h>
#include <ros/package.h>
#include <ros/ros.h>

const static std::string SAMPLE_MESH_SRV_TOPIC = "sample_mesh";
const static double SRV_TIMEOUT = 5.0;
const static std::string INPUT_CLOUD_TOPIC = "input_cloud";
const static std::string SAVED_DB_NAME = "reach.db";
const static std::string OPT_SAVED_DB_NAME = "optimized_reach.db";
const static int MAX_NUM_OPT = 10;
const static double PCT_IMPROVE_THRESHOLD = 0.01;
const static double MAJOR_LENGTH_TO_MARKER_RATIO = 30;

robot_reach_study::ReachStudy::ReachStudy(ros::NodeHandle& nh,
                                          robot_reach_study::StudyParams& sp)
{
  nh_ = nh;
  sp_ = sp;
  helper_.reset(new robot_reach_study::IkHelper (sp_.kin_group_name, sp_.manip_group_name));
  db_.reset(new robot_reach_study::Database);
  ik_visualizer_.reset(new robot_reach_study::InteractiveIK (nh_, db_, helper_));
  cloud_.reset(new pcl::PointCloud<pcl::PointNormal> ());
}

void robot_reach_study::ReachStudy::init()
{
  // Initialize IK parameters
  helper_->setSolutionAttempts(1);
  helper_->setSolutionTimeout(0.02);
  helper_->setNeighborRadius(2.0 * sp_.cloud_output_res);
  helper_->setCostFunction(static_cast<robot_reach_study::IkHelper::CostFunction>(sp_.cost_function));
  helper_->setDistanceThreshold(sp_.distance_threshold);

  // Create a directory to store results of study
  dir_ = ros::package::getPath("robot_reach_study") + "/results/";
  results_dir_ =  dir_ + sp_.config_name + "/";
  const char* char_dir = results_dir_.c_str();

  if(!boost::filesystem::exists(char_dir))
  {
    boost::filesystem::path path(char_dir);
    boost::filesystem::create_directory(path);
  }
}

bool robot_reach_study::ReachStudy::run()
{
  // Initialize the study
  init();

  // Get the reach object point cloud
  if(!getReachObjectPointCloud())
  {
    ROS_ERROR("Unable to obtain reach object point cloud");
    return false;
  }

  // Add the reach object mesh as a collision object in the planning scene
  std::vector<std::string> touch_links;
  touch_links.push_back(sp_.fixed_frame);
  if(!helper_->addCollisionObjectToScene(sp_.object_mesh_filename, sp_.object_frame, touch_links))
  {
    ROS_ERROR("Unable to add collision object to planning scene");
    return false;
  }

  // Show the reach object collision object and reach object point cloud
  if(sp_.visualize_results)
  {
    ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(INPUT_CLOUD_TOPIC, 1, true);
    pub.publish(cloud_msg_);

    moveit_msgs::PlanningScene msg;
    helper_->getPlanningScene()->getPlanningSceneMsg(msg);
    ik_visualizer_->publishScene(msg);
  }

  // Attempt to load previously saved optimized reach_study database
  if(!db_->load(results_dir_ + OPT_SAVED_DB_NAME))
  {
    // Attempt to load previously saved initial reach study database
    if(!db_->load(results_dir_ + SAVED_DB_NAME))
    {
      ROS_INFO("------------------------------");
      ROS_INFO("No reach study database loaded");
      ROS_INFO("------------------------------");

      // Run the first pass of the reach study
      runInitialReachStudy();
      db_->printResults();
    }
    else
    {
      ROS_INFO("----------------------------------------------------");
      ROS_INFO("Unoptimized reach study database successfully loaded");
      ROS_INFO("----------------------------------------------------");

      db_->printResults();
    }

    // Run the optimization
    optimizeReachStudyResults();
    db_->printResults();
  }
  else
  {
    ROS_INFO("--------------------------------------------------");
    ROS_INFO("Optimized reach study database successfully loaded");
    ROS_INFO("--------------------------------------------------");

    db_->printResults();
  }

  // Find the average number of neighboring points can be reached by the robot from any given point
  if(sp_.get_neighbors)
  {
    // Perform the calculation if it hasn't already been done
    if(db_->getAverageNeighborsCount() == 0.0)
    {
      getAverageNeighborsCount();
    }
  }

  // Visualize the results of the reach study
  if(sp_.visualize_results)
  {
    // Compare database results
    if(!sp_.compare_dbs.empty())
    {
      if(!compareDatabases())
      {
        ROS_ERROR("Unable to compare the current reach study database with the other specified databases");
      }
    }

    // Create markers
    ik_visualizer_->setMarkerFrame(sp_.fixed_frame);
    double marker_size = static_cast<double>(utils::getMajorLength(cloud_)) / MAJOR_LENGTH_TO_MARKER_RATIO;
    ik_visualizer_->setMarkerScale(marker_size);
    ik_visualizer_->createReachMarkers();
    ros::spin();
  }

  return true;
}

bool robot_reach_study::ReachStudy::getReachObjectPointCloud()
{
  // Call the sample mesh service to create a point cloud of the reach object mesh
  ros::ServiceClient client = nh_.serviceClient<robot_reach_study::SampleMesh>(SAMPLE_MESH_SRV_TOPIC);

  robot_reach_study::SampleMesh srv;
  srv.request.cloud_filename = sp_.pcd_file;
  srv.request.mesh_filename = sp_.object_mesh_filename;
  srv.request.world_name = sp_.fixed_frame;
  srv.request.object_name = sp_.object_frame;
  srv.request.n_neighbors = sp_.n_neighbors;
  srv.request.output_resolution = sp_.cloud_output_res;

  client.waitForExistence(ros::Duration (SRV_TIMEOUT));
  if(client.call(srv))
  {
    cloud_msg_ = srv.response.cloud;
    pcl::fromROSMsg(cloud_msg_, *cloud_);
  }
  else
  {
    ROS_ERROR("Failed to load reach object point cloud");
    return false;
  }

  cloud_msg_.header.frame_id = sp_.fixed_frame;
  cloud_msg_.header.stamp = ros::Time::now();

  return true;
}

void robot_reach_study::ReachStudy::runInitialReachStudy()
{
  // Loop through all points in point cloud and get IK solution
  std::atomic<int> current_counter, previous_pct;
  current_counter = previous_pct = 0;
  const int cloud_size = static_cast<int>(cloud_->points.size());

  #pragma omp parallel for
  for(int i = 0; i < cloud_size; ++i)
  {
    // Get pose from point cloud array
    const pcl::PointNormal& pt = cloud_->points[i];
    Eigen::Affine3d tgt_frame;
    tgt_frame = utils::createFrame(pt.getArray3fMap(), pt.getNormalVector3fMap());
    geometry_msgs::Pose tgt_pose;
    tf::poseEigenToMsg(tgt_frame, tgt_pose);

    // Create robot state objects for goal and seed to be filled by ik_helper
    moveit::core::RobotState goal_state(helper_->getCurrentRobotState());
    moveit::core::RobotState seed_state(helper_->getCurrentRobotState());

    // Solve IK using setfromIK
    boost::optional<double> score = helper_->solveIKFromSeed(tgt_pose, seed_state, goal_state);
    if(score)
    {
      auto msg = robot_reach_study::makeRecordSuccess(std::to_string(i), tgt_pose, seed_state, goal_state, *score);
      db_->put(msg);
    }
    else
    {
      auto msg = robot_reach_study::makeRecordFailure(std::to_string(i), tgt_pose, seed_state, 0.0);
      db_->put(msg);
    }

    // Print function progress
    current_counter++;
    robot_reach_study::utils::integerProgressPrinter(current_counter, previous_pct, cloud_size);
  }

  // Save the results of the reach study to a database that we can query later
  db_->calculateResults();
  db_->save(results_dir_ + SAVED_DB_NAME);
}

void robot_reach_study::ReachStudy::optimizeReachStudyResults()
{
  ROS_INFO("----------------------");
  ROS_INFO("Beginning optimization");

  // Create sequential vector to be randomized
  std::vector<int> rand_vec;
  for(int i = 0; i < db_->count(); ++i)
  {
    rand_vec.push_back(i);
  }

  // Iterate
  std::atomic<int> current_counter, previous_pct;
  const int cloud_size = static_cast<int>(cloud_->size());
  int n_opt = 0;
  float previous_score = 0.0;
  float pct_improve = 1.0;

  while(pct_improve > PCT_IMPROVE_THRESHOLD && n_opt < MAX_NUM_OPT)
  {
    ROS_INFO("Entering optimization loop %d", n_opt);
    previous_score = db_->getNormalizedTotalPoseScore();
    current_counter = 0;
    previous_pct = 0;

    // Randomize
    std::random_shuffle(rand_vec.begin(), rand_vec.end());

    #pragma parallel for
    for(std::size_t i = 0; i < rand_vec.size(); ++i)
    {
      robot_reach_study::ReachRecord msg = *(db_->get(std::to_string(rand_vec[i])));
      if(msg.reached)
      {
        std::vector<std::string> score = helper_->reachNeighborsDirect(db_, msg);
      }

      // Print function progress
      current_counter++;
      robot_reach_study::utils::integerProgressPrinter(current_counter, previous_pct, cloud_size);
    }

    // Recalculate optimized reach study results
    db_->calculateResults();
    pct_improve = std::abs((db_->getNormalizedTotalPoseScore() - previous_score)/previous_score);
    ++ n_opt;
  }

  // Save the optimized reach database
  db_->calculateResults();
  db_->save(results_dir_ + OPT_SAVED_DB_NAME);

  ROS_INFO("----------------------");
  ROS_INFO("Beginning optimization");
}

void robot_reach_study::ReachStudy::getAverageNeighborsCount()
{
  ROS_INFO("--------------------------------------------");
  ROS_INFO("Beginning average neighbor count calculation");

  // Change the solution planning group
  helper_->setKinematicJointModelGroup(sp_.manip_group_name);

  std::atomic<int> current_counter, previous_pct, neighbor_count;
  current_counter = previous_pct = neighbor_count = 0;
  std::atomic<double> total_joint_distance;
  const int total = db_->count();

  // Iterate
  #pragma parallel for
  for(int i = 0; i < db_->count(); ++i)
  {
    boost::optional<robot_reach_study::ReachRecord> msg = db_->get(std::to_string(i));
    if(msg && msg->reached)
    {
      std::vector<std::string> reached_pts;
      double joint_distance = 0.0;
      helper_->reachNeighborsRecursive(db_, *msg, reached_pts, joint_distance);
      neighbor_count += static_cast<int>(reached_pts.size() - 1);
      total_joint_distance = total_joint_distance + joint_distance;
    }

    // Print function progress
    ++ current_counter;
    robot_reach_study::utils::integerProgressPrinter(current_counter, previous_pct, total);
  }

  float avg_neighbor_count = static_cast<float>(neighbor_count.load()) / static_cast<float>(db_->count());
  float avg_joint_distance = static_cast<float>(total_joint_distance.load()) / static_cast<float>(neighbor_count.load());

  ROS_INFO("Average number of neighbors reached: %f", avg_neighbor_count);
  ROS_INFO("Average joint distance: %f", avg_joint_distance);
  ROS_INFO("------------------------------------------------");

  helper_->setKinematicJointModelGroup(sp_.kin_group_name);

  db_->setAverageNeighborsCount(avg_neighbor_count);
  db_->setAverageJointDistance(avg_joint_distance);
  db_->save(results_dir_ + OPT_SAVED_DB_NAME);
}

bool robot_reach_study::ReachStudy::compareDatabases()
{
  // Add the newly created database to the list if it isn't already there
  if(std::find(sp_.compare_dbs.begin(), sp_.compare_dbs.end(), sp_.config_name) == sp_.compare_dbs.end())
  {
    sp_.compare_dbs.push_back(sp_.config_name);
  }

  // Create list of optimized database file names from the results folder
  std::vector<std::string> db_filenames;
  for(auto it = sp_.compare_dbs.begin(); it != sp_.compare_dbs.end(); ++it)
  {
    db_filenames.push_back(dir_ + *it + "/" + OPT_SAVED_DB_NAME);
  }

  // Load databases to be compared
  std::vector<std::pair<std::string, std::shared_ptr<robot_reach_study::Database>>> data;
  for(size_t i = 0; i < db_filenames.size(); ++i)
  {
    std::shared_ptr<robot_reach_study::Database> db (new robot_reach_study::Database);
    if(!db->load(db_filenames[i]))
    {
      ROS_ERROR("Cannot load database at:\n %s", db_filenames[i].c_str());
      continue;
    }
    std::pair<std::string, std::shared_ptr<robot_reach_study::Database>> pair (sp_.compare_dbs[i], db);
    data.push_back(pair);
  }

  if(data.size() < 2)
  {
    ROS_ERROR("Only %lu database(s) loaded; cannot compare fewer than 2 databases", data.size());
    return false;
  }

  ik_visualizer_->reachDiffVisualizer(data);

  return true;
}
