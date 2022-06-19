/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <reach_core/reach_study.h>
#include <reach_core/utils/general_utils.h>

#include <numeric>
#include <pluginlib/class_loader.h>
#include <thread>

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
static const std::string DISPLAY_BASE_CLASS = "reach::plugins::DisplayBase";
static const std::string TARGET_POSE_GENERATOR_BASE_CLASS = "reach::plugins::WaypointGeneratorBase";

ReachStudy::ReachStudy()
  : db_(new ReachDatabase())
  , solver_loader_(PACKAGE, IK_BASE_CLASS)
  , display_loader_(PACKAGE, DISPLAY_BASE_CLASS)
  , target_pose_generator_loader_(PACKAGE, TARGET_POSE_GENERATOR_BASE_CLASS)
{
}

void ReachStudy::initializeStudy()
{
  // Initialize the IK solver plugin and display plugin
  ik_solver_ = solver_loader_.createInstance(sp_.ik_solver_config["name"]);
  ik_solver_->initialize(sp_.ik_solver_config);

  display_ = display_loader_.createInstance(sp_.display_config["name"]);
  display_->initialize(sp_.display_config);
  display_->showEnvironment();

  auto waypoint_generator = target_pose_generator_loader_.createInstance(sp_.target_pose_generator_config["name"]);
  waypoint_generator->initialize(sp_.target_pose_generator_config);
  target_poses_ = waypoint_generator->generate();

  // Create a directory to store results of study
  if (!sp_.results_directory.empty())
  {
    dir_ = sp_.results_directory + "/";
  }
  else
  {
    dir_ = ros::package::getPath("reach_core") + "/results/";
    ROS_WARN_STREAM("Using default results file directory: '" << dir_ << "'");
  }
  results_dir_ = dir_ + sp_.config_name + "/";

  if (!boost::filesystem::exists(results_dir_))
    boost::filesystem::create_directories(results_dir_);
}

void ReachStudy::run(const StudyParameters& sp)
{
  // Overrwrite the old study parameters
  sp_ = sp;

  // Initialize the study
  initializeStudy();

  // Create markers
  visualizer_ = boost::make_shared<ReachVisualizer>(db_, ik_solver_, display_, sp_.optimization.radius);

  // Attempt to load previously saved optimized reach_study database
  try
  {
    *db_ = load(results_dir_ + OPT_SAVED_DB_NAME);

    ROS_INFO("--------------------------------------------------");
    ROS_INFO("Optimized reach study database successfully loaded");
    ROS_INFO("--------------------------------------------------");

    db_->printResults();
    visualizer_->update();
  }
  catch (const std::exception&)
  {
    try
    {
      // Attempt to load previously saved initial reach study database
      *db_ = load(results_dir_ + OPT_SAVED_DB_NAME);

      ROS_INFO("----------------------------------------------------");
      ROS_INFO("Unoptimized reach study database successfully loaded");
      ROS_INFO("----------------------------------------------------");

      db_->printResults();
      visualizer_->update();
    }
    catch(const std::exception&)
    {
      ROS_INFO("------------------------------");
      ROS_INFO("No reach study database loaded");
      ROS_INFO("------------------------------");

      // Run the first pass of the reach study
      runInitialReachStudy();
      db_->printResults();
      visualizer_->update();
    }

    // Create an efficient search tree for doing nearest neighbors search
    {
      auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      for (auto it = db_->begin(); it != db_->end(); ++it)
      {
        pcl::PointXYZ pt(it->second.goal.translation().x(), it->second.goal.translation().y(),
                         it->second.goal.translation().z());
        cloud->push_back(pt);
      }
      search_tree_ = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
      search_tree_->setInputCloud(cloud);
    }

    // Run the optimization
    optimizeReachStudyResults();
    db_->printResults();
    visualizer_->update();
  }

  // Find the average number of neighboring points can be reached by the robot from any given point
  if (sp_.get_neighbors)
  {
    // Perform the calculation if it hasn't already been done
    if (db_->getStudyResults().avg_num_neighbors == 0.0f)
    {
      getAverageNeighborsCount();
    }
  }

  // Visualize the results of the reach study
  if (sp_.visualize_results)
  {
    // Compare database results
    if (!sp_.compare_dbs.empty())
    {
      if (!compareDatabases())
      {
        ROS_ERROR("Unable to compare the current reach study database with the other specified databases");
      }
    }
  }
}

void ReachStudy::runInitialReachStudy()
{
  // Loop through all points in point cloud and get IK solution
  std::atomic<int> current_counter, previous_pct;
  current_counter = previous_pct = 0;

#pragma omp parallel for num_threads(std::thread::hardware_concurrency())
  for (std::size_t i = 0; i < target_poses_.size(); ++i)
  {
    const Eigen::Isometry3d& tgt_frame = target_poses_[i];

    // Get the seed position
    const std::vector<std::string> joint_names = ik_solver_->getJointNames();
    std::map<std::string, double> seed_state = utils::zip(joint_names, std::vector<double>(joint_names.size(), 0.0));

    // Solve IK
    try
    {
      std::vector<double> solution;
      double score;
      std::tie(solution, score) = ik_solver_->solveIKFromSeed(tgt_frame, seed_state);

      ReachRecord msg(std::to_string(i), true, tgt_frame, seed_state, utils::zip(ik_solver_->getJointNames(), solution), score);
      db_->put(msg);
    }
    catch(const std::exception&)
    {
      ReachRecord msg(std::to_string(i), false, tgt_frame, seed_state, seed_state, 0.0);
      db_->put(msg);
    }

    // Print function progress
    current_counter++;
    utils::integerProgressPrinter(current_counter, previous_pct, target_poses_.size());
  }

  // Save the results of the reach study to a database that we can query later
  db_->calculateResults();
  save(*db_, results_dir_ + SAVED_DB_NAME);
}

void ReachStudy::optimizeReachStudyResults()
{
  ROS_INFO("----------------------");
  ROS_INFO("Beginning optimization");

  // Create sequential vector to be randomized
  std::vector<std::size_t> rand_vec(db_->size());
  std::iota(rand_vec.begin(), rand_vec.end(), 0);

  // Iterate
  std::atomic<int> current_counter, previous_pct;
  int n_opt = 0;
  float previous_score = 0.0;
  float pct_improve = 1.0;

  while (pct_improve > sp_.optimization.step_improvement_threshold && n_opt < sp_.optimization.max_steps)
  {
    ROS_INFO("Entering optimization loop %d", n_opt);
    previous_score = db_->getStudyResults().norm_total_pose_score;
    current_counter = 0;
    previous_pct = 0;

    // Randomize
    std::random_shuffle(rand_vec.begin(), rand_vec.end());

#pragma parallel for num_threads(std::thread::hardware_concurrency())
    for (std::size_t i = 0; i < rand_vec.size(); ++i)
    {
      auto it = db_->begin();
      std::advance(it, rand_vec[i]);
      ReachRecord msg = it->second;
      if (msg.reached)
      {
        NeighborReachResult result = reachNeighborsDirect(db_, msg, ik_solver_, sp_.optimization.radius, search_tree_);
      }

      // Print function progress
      current_counter++;
      utils::integerProgressPrinter(current_counter, previous_pct, target_poses_.size());
    }

    // Recalculate optimized reach study results
    db_->calculateResults();
    db_->printResults();
    pct_improve = std::abs((db_->getStudyResults().norm_total_pose_score - previous_score) / previous_score);
    ++n_opt;
  }

  // Save the optimized reach database
  db_->calculateResults();
  save(*db_, results_dir_ + OPT_SAVED_DB_NAME);

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
#pragma parallel for num_threads(std::thread::hardware_concurrency())
  for (auto it = db_->begin(); it != db_->end(); ++it)
  {
    ReachRecord msg = it->second;
    if (msg.reached)
    {
      NeighborReachResult result;
      reachNeighborsRecursive(db_, msg, ik_solver_, sp_.optimization.radius, result, search_tree_);

      neighbor_count += static_cast<int>(result.reached_pts.size() - 1);
      total_joint_distance = total_joint_distance + result.joint_distance;
    }

    // Print function progress
    ++current_counter;
    utils::integerProgressPrinter(current_counter, previous_pct, total);
  }

  float avg_neighbor_count = static_cast<float>(neighbor_count.load()) / static_cast<float>(db_->size());
  float avg_joint_distance =
      static_cast<float>(total_joint_distance.load()) / static_cast<float>(neighbor_count.load());

  ROS_INFO_STREAM("Average number of neighbors reached: " << avg_neighbor_count);
  ROS_INFO_STREAM("Average joint distance: " << avg_joint_distance);
  ROS_INFO("------------------------------------------------");

  db_->setAverageNeighborsCount(avg_neighbor_count);
  db_->setAverageJointDistance(avg_joint_distance);
  save(*db_, results_dir_ + OPT_SAVED_DB_NAME);
}

bool ReachStudy::compareDatabases()
{
  // Add the newly created database to the list if it isn't already there
  if (std::find(sp_.compare_dbs.begin(), sp_.compare_dbs.end(), sp_.config_name) == sp_.compare_dbs.end())
  {
    sp_.compare_dbs.push_back(sp_.config_name);
  }

  // Create list of optimized database file names from the results folder
  std::vector<std::string> db_filenames;
  for (auto it = sp_.compare_dbs.begin(); it != sp_.compare_dbs.end(); ++it)
  {
    db_filenames.push_back(dir_ + *it + "/" + OPT_SAVED_DB_NAME);
  }

  // Load databases to be compared
  std::map<std::string, ReachDatabase> data;
  for (size_t i = 0; i < db_filenames.size(); ++i)
  {
    try
    {
      data.emplace(sp_.compare_dbs[i], load(db_filenames[i]));
    }
    catch (const std::exception&)
    {
      continue;
    }
  }

  if (data.size() < 2)
  {
    ROS_ERROR("Only %lu database(s) loaded; cannot compare fewer than 2 databases", data.size());
    return false;
  }

  display_->compareDatabases(data);

  return true;
}

}  // namespace core
}  // namespace reach
