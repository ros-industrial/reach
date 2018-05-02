#include "reach/core/reach_study.h"
#include "reach/utils/database_utils.h"
#include "reach/utils/general_utils.h"
#include "reach/utils/kinematics_utils.h"

#include <reach_msgs/SampleMesh.h>
#include <reach_msgs/ReachRecord.h>

#include <eigen_conversions/eigen_msg.h>
#include <ros/package.h>

const static std::string SAMPLE_MESH_SRV_TOPIC = "sample_mesh";
const static double SRV_TIMEOUT = 5.0;
const static std::string INPUT_CLOUD_TOPIC = "input_cloud";
const static std::string SAVED_DB_NAME = "reach.db";
const static std::string OPT_SAVED_DB_NAME = "optimized_reach.db";
const static int MAX_NUM_OPT = 10;
const static double PCT_IMPROVE_THRESHOLD = 0.01;

const static int SOLUTION_ATTEMPTS = 1;
const static float SOLUTION_TIMEOUT = 0.02;
const static double DISCRETIZATION_ANGLE = 10.0f * (M_PI / 180.0f);

namespace reach
{
namespace core
{

ReachStudy::ReachStudy(const ros::NodeHandle& nh,
                       const StudyParameters& sp)
  : nh_(nh)
  , sp_(sp)
  , helper_(new IkHelper (sp.kin_group_name, sp.manip_group_name))
  , db_(new ReachDatabase ())
  , visualizer_(new ReachVisualizer (nh_, db_, helper_))
  , cloud_(new pcl::PointCloud<pcl::PointNormal> ())
{

}

void ReachStudy::initializeStudy(const StudyParameters& sp)
{
  // Set the IK parameters
  helper_->setSolutionAttempts(SOLUTION_ATTEMPTS);
  helper_->setSolutionTimeout(SOLUTION_TIMEOUT);
  helper_->setNeighborRadius(sp.optimization_radius);
  helper_->setCostFunction(static_cast<CostFunction>(sp.cost_function));
  helper_->setDistanceThreshold(sp.distance_threshold);

  // Remove seed states whose length is not equal to the number of joints in the kinematic chain
  const std::size_t n_joints = helper_->getKinematicGroupJointCount();
  auto it = std::remove_if(sp_.seed_states.begin(), sp_.seed_states.end(), [&n_joints](const std::vector<double>& state)
  {
    return state.size() != n_joints;
  });
  sp_.seed_states.erase(it, sp_.seed_states.end());

  // Add an all zeros seed state if the seed state vector ends up being empty
  if(sp_.seed_states.empty())
  {
    std::vector<double> seed (helper_->getKinematicGroupJointCount(), 0.0f);
    sp_.seed_states.push_back(std::move(seed));
  }

  // Set the visualizer parameters
  visualizer_->setMarkerFrame(sp.fixed_frame);
  visualizer_->setMarkerScale(sp.optimization_radius / 2.0);

  // Create a directory to store results of study
  if(!sp.results_directory.empty() && boost::filesystem::exists(sp.results_directory.c_str()))
  {
    dir_ = sp.results_directory + "/";
  }
  else
  {
    dir_ = ros::package::getPath("reach_core") + "/results/";
    ROS_WARN("Using default results file directory: %s", dir_.c_str());
  }
  results_dir_ =  dir_ + sp.config_name + "/";
  const char* char_dir = results_dir_.c_str();

  if(!boost::filesystem::exists(char_dir))
  {
    boost::filesystem::path path(char_dir);
    boost::filesystem::create_directory(path);
  }
}

bool ReachStudy::run(const StudyParameters& sp)
{
  // Initialize the study
  initializeStudy(sp);

  // Get the reach object point cloud
  if(!getReachObjectPointCloud())
  {
    ROS_ERROR("Unable to obtain reach object point cloud");
    return false;
  }

  // Add the reach object mesh as a collision object in the planning scene
  std::vector<std::string> touch_links;
  touch_links.push_back(sp.fixed_frame);
  if(!helper_->addCollisionObjectToScene(sp.mesh_filename, sp.object_frame, touch_links))
  {
    ROS_ERROR("Unable to add collision object to planning scene");
    return false;
  }

  // Show the reach object collision object and reach object point cloud
  if(sp.visualize_results)
  {
    ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(INPUT_CLOUD_TOPIC, 1, true);
    pub.publish(cloud_msg_);

    moveit_msgs::PlanningScene msg;
    helper_->getPlanningScene()->getPlanningSceneMsg(msg);
    visualizer_->publishScene(msg);
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
  if(sp.get_neighbors)
  {
    // Perform the calculation if it hasn't already been done
    if(db_->getStudyResults().avg_num_neighbors == 0.0)
    {
      getAverageNeighborsCount();
    }
  }

  // Visualize the results of the reach study
  if(sp.visualize_results)
  {
    // Compare database results
    if(!sp.compare_dbs.empty())
    {
      if(!compareDatabases())
      {
        ROS_ERROR("Unable to compare the current reach study database with the other specified databases");
      }
    }

    // Create markers
    visualizer_->createReachMarkers();
    ros::spin();
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
  // Rotation to flip the Z axis fo the surface normal point
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

    // Solve IK using setfromIKDiscretized
    std::vector<reach_msgs::ReachRecord> records (sp_.seed_states.size());
    for(std::size_t j = 0; j < sp_.seed_states.size(); ++j)
    {
      // Create robot state objects for goal and seed to be filled by ik_helper
      moveit::core::RobotState seed_state(helper_->getCurrentRobotState());
      seed_state.setJointGroupPositions(sp_.kin_group_name, sp_.seed_states[j]);
      seed_state.update();
      moveit::core::RobotState goal_state(seed_state);

      boost::optional<double> score = helper_->solveDiscretizedIKFromSeed(tgt_frame, DISCRETIZATION_ANGLE, seed_state, goal_state);
      geometry_msgs::Pose tgt_pose;
      tf::poseEigenToMsg(tgt_frame, tgt_pose);

      if(score)
      {
        records[j] = utils::makeRecord(std::to_string(i), true, tgt_pose, seed_state, goal_state, *score);
      }
      else
      {
        records[j] = utils::makeRecord(std::to_string(i), false, tgt_pose, seed_state, goal_state, 0.0f);
      }
    }

    std::sort(records.begin(), records.end(), [](const reach_msgs::ReachRecord& a, const reach_msgs::ReachRecord& b)
    {
      return a.score > b.score;
    });

    db_->put(records.front());

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
    previous_score = db_->getStudyResults().norm_total_pose_score;
    current_counter = 0;
    previous_pct = 0;

    // Randomize
    std::random_shuffle(rand_vec.begin(), rand_vec.end());

    #pragma parallel for
    for(std::size_t i = 0; i < rand_vec.size(); ++i)
    {
      reach_msgs::ReachRecord msg = *(db_->get(std::to_string(rand_vec[i])));
      if(msg.reached)
      {
        std::vector<std::string> score = helper_->reachNeighborsDirect(db_, msg);
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
    boost::optional<reach_msgs::ReachRecord> msg = db_->get(std::to_string(i));
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
    utils::integerProgressPrinter(current_counter, previous_pct, total);
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
  std::vector<std::pair<std::string, std::shared_ptr<ReachDatabase>>> data;
  for(size_t i = 0; i < db_filenames.size(); ++i)
  {
    std::shared_ptr<ReachDatabase> db (new ReachDatabase);
    if(!db->load(db_filenames[i]))
    {
      ROS_ERROR("Cannot load database at:\n %s", db_filenames[i].c_str());
      continue;
    }
    std::pair<std::string, std::shared_ptr<ReachDatabase>> pair (sp_.compare_dbs[i], db);
    data.push_back(pair);
  }

  if(data.size() < 2)
  {
    ROS_ERROR("Only %lu database(s) loaded; cannot compare fewer than 2 databases", data.size());
    return false;
  }

  visualizer_->reachDiffVisualizer(data);

  return true;
}

} // namespace core
} // namespace reach
