#include <reach/core/reach_study.h>
#include <reach/utils/serialization_utils.h>
#include <reach/utils/general_utils.h>

#include <reach_msgs/SampleMesh.h>
#include <reach_msgs/ReachRecord.h>

#include <eigen_conversions/eigen_msg.h>
#include <pluginlib/class_loader.h>
#include <ros/package.h>
#include <xmlrpcpp/XmlRpcException.h>

const static std::string SAMPLE_MESH_SRV_TOPIC = "sample_mesh";
const static double SRV_TIMEOUT = 5.0;
const static std::string INPUT_CLOUD_TOPIC = "input_cloud";
const static std::string SAVED_DB_NAME = "reach.db";
const static std::string OPT_SAVED_DB_NAME = "optimized_reach.db";

namespace reach
{
namespace core
{

static const std::string PACKAGE = "reach_core";
static const std::string IK_BASE_CLASS = "reach::plugins::IKSolverBase";
static const std::string DISPLAY_BASE_CLASS = "reach::plugins::ReachDisplayBase";

ReachStudy::ReachStudy(const ros::NodeHandle& nh)
  : nh_(nh)
  , cloud_(new pcl::PointCloud<pcl::PointNormal> ())
  , db_(new ReachDatabase ())
  , solver_loader_(PACKAGE, IK_BASE_CLASS)
  , display_loader_(PACKAGE, DISPLAY_BASE_CLASS)
{

}

bool ReachStudy::initializeStudy()
{
  ik_solver_.reset();
  display_.reset();

  try
  {
    ik_solver_ = solver_loader_.createInstance(sp_.ik_solver_config["name"]);
    display_ = display_loader_.createInstance(sp_.display_config["name"]);
  }
  catch(const XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR_STREAM(ex.getMessage());
    return false;
  }
  catch(const pluginlib::PluginlibException& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }

  // Initialize the IK solver plugin and display plugin
  if(!ik_solver_->initialize(sp_.ik_solver_config) ||
     !display_->initialize(sp_.display_config))
  {
    return false;
  }

  display_->showEnvironment();

  // Create a directory to store results of study
  if(!sp_.results_directory.empty() && boost::filesystem::exists(sp_.results_directory.c_str()))
  {
    dir_ = sp_.results_directory + "/";
  }
  else
  {
    dir_ = ros::package::getPath("reach_core") + "/results/";
    ROS_WARN("Using default results file directory: %s", dir_.c_str());
  }
  results_dir_ =  dir_ + sp_.config_name + "/";
  const char* char_dir = results_dir_.c_str();

  if(!boost::filesystem::exists(char_dir))
  {
    boost::filesystem::path path(char_dir);
    boost::filesystem::create_directory(path);
  }

  return true;
}

bool ReachStudy::run(const StudyParameters& sp)
{
  // Overrwrite the old study parameters
  sp_ = sp;

  // Initialize the study
  if(!initializeStudy())
  {
    ROS_ERROR("Failed to initialize the reach study");
    return false;
  }

  // Get the reach object point cloud
  if(!getReachObjectPointCloud())
  {
    ROS_ERROR("Unable to obtain reach object point cloud");
    return false;
  }

  // Show the reach object collision object and reach object point cloud
  if(sp_.visualize_results)
  {
    ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(INPUT_CLOUD_TOPIC, 1, true);
    pub.publish(cloud_msg_);
  }

  // Create markers
  visualizer_.reset(new ReachVisualizer(db_, ik_solver_, display_, sp_.optimization.radius));

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
      visualizer_->update();
    }
    else
    {
      ROS_INFO("----------------------------------------------------");
      ROS_INFO("Unoptimized reach study database successfully loaded");
      ROS_INFO("----------------------------------------------------");

      db_->printResults();
      visualizer_->update();
    }

    // Create an efficient search tree for doing nearest neighbors search
    search_tree_.reset(new SearchTree(flann::KDTreeSingleIndexParams(1, true)));

    flann::Matrix<double> dataset (new double[db_->size() * 3], db_->size(), 3);
    for(std::size_t i = 0; i < db_->size(); ++i)
    {
      auto it = db_->begin();
      std::advance(it, i);

      dataset[i][0] = static_cast<double>(it->second.goal.position.x);
      dataset[i][1] = static_cast<double>(it->second.goal.position.y);
      dataset[i][2] = static_cast<double>(it->second.goal.position.z);
    }
    search_tree_->buildIndex(dataset);

    // Run the optimization
    optimizeReachStudyResults();
    db_->printResults();
    visualizer_->update();
  }
  else
  {
    ROS_INFO("--------------------------------------------------");
    ROS_INFO("Optimized reach study database successfully loaded");
    ROS_INFO("--------------------------------------------------");

    db_->printResults();
    visualizer_->update();
  }

  // Find the average number of neighboring points can be reached by the robot from any given point
  if(sp_.get_neighbors)
  {
    // Perform the calculation if it hasn't already been done
    if(db_->getStudyResults().avg_num_neighbors == 0.0f)
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
  }

  return true;
}

bool ReachStudy::getReachObjectPointCloud()
{
  // Call the sample mesh service to create a point cloud of the reach object mesh
  ros::ServiceClient client = nh_.serviceClient<reach_msgs::SampleMesh>(SAMPLE_MESH_SRV_TOPIC);

  reach_msgs::SampleMesh srv;
  srv.request.cloud_filename = sp_.pcd_filename;
  srv.request.fixed_frame = sp_.fixed_frame;
  srv.request.object_frame = sp_.object_frame;

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

void ReachStudy::runInitialReachStudy()
{
  // Rotation to flip the Z axis of the surface normal point
  const Eigen::AngleAxisd tool_z_rot(M_PI, Eigen::Vector3d::UnitY());

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
    tgt_frame = tgt_frame * tool_z_rot;

    // Get the seed position
    sensor_msgs::JointState seed_state;
    seed_state.name = ik_solver_->getJointNames();
    seed_state.position = std::vector<double>(seed_state.name.size(), 0.0);

    // Solve IK
    std::vector<double> solution;
    boost::optional<double> score = ik_solver_->solveIKFromSeed(tgt_frame, jointStateMsgToMap(seed_state), solution);

    // Create objects to save in the reach record
    geometry_msgs::Pose tgt_pose;
    tf::poseEigenToMsg(tgt_frame, tgt_pose);

    sensor_msgs::JointState goal_state (seed_state);

    if(score)
    {
      goal_state.position = solution;
      auto msg = makeRecord(std::to_string(i), true, tgt_pose, seed_state, goal_state, *score);
      db_->put(msg);
    }
    else
    {
      auto msg = makeRecord(std::to_string(i), false, tgt_pose, seed_state, goal_state, 0.0);
      db_->put(msg);
    }

    // Print function progress
    current_counter++;
    utils::integerProgressPrinter(current_counter, previous_pct, cloud_size);
  }

  // Save the results of the reach study to a database that we can query later
  db_->calculateResults();
  db_->save(results_dir_ + SAVED_DB_NAME);
}

void ReachStudy::optimizeReachStudyResults()
{
  ROS_INFO("----------------------");
  ROS_INFO("Beginning optimization");

  // Create sequential vector to be randomized
  std::vector<std::size_t> rand_vec (db_->size());
  std::iota(rand_vec.begin(), rand_vec.end(), 0);

  // Iterate
  std::atomic<int> current_counter, previous_pct;
  const int cloud_size = static_cast<int>(cloud_->size());
  int n_opt = 0;
  float previous_score = 0.0;
  float pct_improve = 1.0;

  while(pct_improve > sp_.optimization.step_improvement_threshold && n_opt < sp_.optimization.max_steps)
  {
    ROS_INFO("Entering optimization loop %d", n_opt);
    previous_score = db_->getStudyResults().norm_total_pose_score;
    current_counter = 0;
    previous_pct = 0;

    // Randomize
    std::random_shuffle(rand_vec.begin(), rand_vec.end());

    #pragma parallel for
    for(std::size_t i = 0; i < rand_vec.size(); ++i)
    {
      auto it = db_->begin();
      std::advance(it, rand_vec[i]);
      reach_msgs::ReachRecord msg = it->second;
      if(msg.reached)
      {
        NeighborReachResult result = reachNeighborsDirect(db_, msg, ik_solver_, sp_.optimization.radius); //, search_tree_);
      }

      // Print function progress
      current_counter++;
      utils::integerProgressPrinter(current_counter, previous_pct, cloud_size);
    }

    // Recalculate optimized reach study results
    db_->calculateResults();
    db_->printResults();
    pct_improve = std::abs((db_->getStudyResults().norm_total_pose_score - previous_score)/previous_score);
    ++ n_opt;
  }

  // Save the optimized reach database
  db_->calculateResults();
  db_->save(results_dir_ + OPT_SAVED_DB_NAME);

  ROS_INFO("----------------------");
  ROS_INFO("Optimization concluded");
}

void ReachStudy::getAverageNeighborsCount()
{
  ROS_INFO("--------------------------------------------");
  ROS_INFO("Beginning average neighbor count calculation");

  std::atomic<int> current_counter, previous_pct, neighbor_count;
  current_counter = previous_pct = neighbor_count = 0;
  std::atomic<double> total_joint_distance;
  const int total = db_->size();

  // Iterate
  #pragma parallel for
  for(auto it = db_->begin(); it != db_->end(); ++it)
  {
    reach_msgs::ReachRecord msg = it->second;
    if(msg.reached)
    {
      NeighborReachResult result;
      reachNeighborsRecursive(db_, msg, ik_solver_, sp_.optimization.radius, result); //, search_tree_);

      neighbor_count += static_cast<int>(result.reached_pts.size() - 1);
      total_joint_distance = total_joint_distance + result.joint_distance;
    }

    // Print function progress
    ++ current_counter;
    utils::integerProgressPrinter(current_counter, previous_pct, total);
  }

  float avg_neighbor_count = static_cast<float>(neighbor_count.load()) / static_cast<float>(db_->size());
  float avg_joint_distance = static_cast<float>(total_joint_distance.load()) / static_cast<float>(neighbor_count.load());

  ROS_INFO_STREAM("Average number of neighbors reached: " << avg_neighbor_count);
  ROS_INFO_STREAM("Average joint distance: " << avg_joint_distance);
  ROS_INFO("------------------------------------------------");

  db_->setAverageNeighborsCount(avg_neighbor_count);
  db_->setAverageJointDistance(avg_joint_distance);
  db_->save(results_dir_ + OPT_SAVED_DB_NAME);
}

bool ReachStudy::compareDatabases()
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
  std::map<std::string, reach_msgs::ReachDatabase> data;
  for(size_t i = 0; i < db_filenames.size(); ++i)
  {
    ReachDatabase db;
    if(!db.load(db_filenames[i]))
    {
      ROS_ERROR("Cannot load database at:\n %s", db_filenames[i].c_str());
      continue;
    }
    data.emplace(sp_.compare_dbs[i], db.toReachDatabaseMsg());
  }

  if(data.size() < 2)
  {
    ROS_ERROR("Only %lu database(s) loaded; cannot compare fewer than 2 databases", data.size());
    return false;
  }

  display_->compareDatabases(data);

  return true;
}

} // namespace core
} // namespace reach
