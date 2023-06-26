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
#include <reach/reach_study.h>
#include <reach/utils.h>
#include <reach/plugin_utils.h>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost_plugin_loader/plugin_loader.hpp>
#include <numeric>
#include <signal.h>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace reach
{
ReachStudy::ReachStudy(IKSolver::ConstPtr ik_solver, Evaluator::ConstPtr evaluator,
                       TargetPoseGenerator::ConstPtr target_generator, Display::ConstPtr display, Logger::Ptr logger,
                       Parameters params)
  : params_(std::move(params))
  , ik_solver_(std::move(ik_solver))
  , evaluator_(std::move(evaluator))
  , display_(std::move(display))
  , logger_(std::move(logger))
  , target_poses_(target_generator->generate())
  , search_tree_(createSearchTree(target_poses_))
{
}

ReachStudy::ReachStudy(const ReachStudy& rhs)
  : params_(rhs.params_)
  , ik_solver_(rhs.ik_solver_)
  , evaluator_(rhs.evaluator_)
  , display_(rhs.display_)
  , logger_(rhs.logger_)
  , target_poses_(rhs.target_poses_)
  , search_tree_(rhs.search_tree_)
{
}

void ReachStudy::load(const std::string& filename)
{
  db_ = reach::load(filename);
  display_->showEnvironment();
  display_->showResults(db_.results.back());
}

void ReachStudy::save(const std::string& filename) const
{
  reach::save(db_, filename);
}

const ReachDatabase& ReachStudy::getDatabase() const
{
  return db_;
}

void ReachStudy::run()
{
  // Clear the database results and add an initial empty reach result
  db_.results.clear();
  db_.results.push_back(ReachResult{ target_poses_.size() });
  auto active_result = db_.results.rbegin();

  logger_->print("Starting reach study");
  logger_->setMaxProgress(target_poses_.size());

  // Show display
  display_->showEnvironment();
  display_->showResults(*active_result);

  // First loop through all points in point cloud and get IK solution
  std::atomic<unsigned long> current_counter;
  current_counter = 0;

#pragma omp parallel for num_threads(params_.max_threads)
  for (std::size_t i = 0; i < target_poses_.size(); ++i)
  {
    const Eigen::Isometry3d& tgt_frame = target_poses_[i] * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

    // Solve IK
    try
    {
      std::vector<double> solution;
      double score;
      std::tie(solution, score) = evaluateIK(tgt_frame, params_.seed_state, ik_solver_, evaluator_);

      ReachRecord msg(true, tgt_frame, params_.seed_state, zip(ik_solver_->getJointNames(), solution), score);
      {
        std::lock_guard<std::mutex> lock{ mutex_ };
        active_result->operator[](i) = msg;
      }
    }
    catch (const std::exception&)
    {
      ReachRecord msg(false, tgt_frame, params_.seed_state, params_.seed_state, 0.0);
      {
        std::lock_guard<std::mutex> lock{ mutex_ };
        active_result->operator[](i) = msg;
      }
    }

    // Print function progress
    current_counter++;
    logger_->printProgress(current_counter.load());
  }

  logger_->printResults(db_.calculateResults());
  logger_->print("Reach study complete");
  display_->showResults(*active_result);
}

void ReachStudy::optimize()
{
  if (db_.results.empty())
    throw std::runtime_error("Database contains no results to optimize");

  auto active_result = db_.results.rbegin();

  logger_->print("Starting optimization");

  // Show environment display
  display_->showEnvironment();
  display_->showResults(*active_result);

  // Create sequential vector to be randomized
  std::vector<std::size_t> rand_vec(active_result->size());
  std::iota(rand_vec.begin(), rand_vec.end(), 0);

  // Iterate
  std::atomic<unsigned long> current_counter;
  int n_opt = 0;
  float previous_score = 0.0;
  float pct_improve = 1.0;

  while (pct_improve > params_.step_improvement_threshold && n_opt < params_.max_steps)
  {
    // Add a new reach result for this iteration of optimization
    db_.results.push_back(*active_result);
    active_result = db_.results.rbegin();

    logger_->print("Entering optimization loop " + std::to_string(n_opt));
    logger_->setMaxProgress(target_poses_.size());

    previous_score = db_.calculateResults().norm_total_pose_score;
    current_counter = 0;

    // Randomize
    std::random_shuffle(rand_vec.begin(), rand_vec.end());

#pragma omp parallel for num_threads(params_.max_threads)
    for (std::size_t i = 0; i < rand_vec.size(); ++i)
    {
      const ReachRecord& msg = active_result->at(i);
      if (msg.reached)
      {
        std::map<std::size_t, ReachRecord> neighbors =
            reachNeighborsDirect(*active_result, msg, ik_solver_, evaluator_, params_.radius, search_tree_);

        // Replace the old records if the scores of the new records are higher
        for (auto neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor)
        {
          const ReachRecord& old_rec = active_result->at(neighbor->first);
          if (neighbor->second.score > old_rec.score)
          {
            std::lock_guard<std::mutex> lock{ mutex_ };
            active_result->operator[](neighbor->first) = neighbor->second;
          }
        }
      }

      // Print function progress
      current_counter++;
      logger_->printProgress(current_counter.load());
    }

    // Recalculate optimized reach study results
    auto results = db_.calculateResults();
    logger_->printResults(results);

    pct_improve = std::abs((results.norm_total_pose_score - previous_score) / previous_score);
    ++n_opt;

    // Show the results
    display_->showResults(*active_result);
  }

  logger_->print("Optimization complete");
}

std::tuple<double, double> ReachStudy::getAverageNeighborsCount() const
{
  if (db_.results.empty())
    throw std::runtime_error("Database contains no results");
  const ReachResult& active_result = db_.results.back();

  logger_->print("Beginning average neighbor count calculation");

  std::atomic<unsigned long> current_counter, neighbor_count;
  current_counter = neighbor_count = 0;
  std::atomic<double> total_joint_distance;

// Iterate
#pragma omp parallel for num_threads(params_.max_threads)
  for (auto it = active_result.begin(); it < active_result.end(); ++it)
  {
    if (it->reached)
    {
      NeighborReachResult result;
      reachNeighborsRecursive(active_result, *it, ik_solver_, evaluator_, params_.radius, result, search_tree_);

      neighbor_count += static_cast<int>(result.reached_pts.size() - 1);
      total_joint_distance = total_joint_distance + result.joint_distance;
    }

    // Print function progress
    ++current_counter;
    logger_->printProgress(current_counter.load());
  }

  float avg_neighbor_count = static_cast<float>(neighbor_count.load()) / static_cast<float>(active_result.size());
  float avg_joint_distance =
      static_cast<float>(total_joint_distance.load()) / static_cast<float>(neighbor_count.load());

  return std::make_tuple(avg_neighbor_count, avg_joint_distance);
}

void runReachStudy(const YAML::Node& config, const std::string& config_name, const boost::filesystem::path& results_dir,
                   const bool wait_after_completion)
{
  const YAML::Node& opt_config = config["optimization"];
  const YAML::Node& ik_config = config["ik_solver"];
  const YAML::Node& pose_gen_config = config["target_pose_generator"];
  const YAML::Node& eval_config = config["evaluator"];
  const YAML::Node& display_config = config["display"];
  const YAML::Node& logger_config = config["logger"];

  // Extract the study parameters
  reach::ReachStudy::Parameters params;
  params.radius = opt_config["radius"].as<double>();
  params.max_steps = opt_config["max_steps"].as<int>();
  params.step_improvement_threshold = opt_config["step_improvement_threshold"].as<double>();
  if (opt_config["max_threads"])
    params.max_threads = opt_config["max_threads"].as<std::size_t>();

  boost_plugin_loader::PluginLoader loader;
  std::vector<std::string> plugin_libraries;
  boost::split(loader.search_libraries, PLUGIN_LIBRARIES, boost::is_any_of(":"), boost::token_compress_on);
  loader.search_libraries_env = SEARCH_LIBRARIES_ENV;

  // Load the IK Solver plugin
  reach::IKSolver::ConstPtr ik_solver;
  {
    auto factory = loader.createInstance<IKSolverFactory>(get<std::string>(ik_config, "name"));
    ik_solver = factory->create(ik_config);
  }

  // read the initial joint positions if specified
  const YAML::Node seed_state_config = opt_config["seed_state"];
  if (!seed_state_config)
  {
    // Initialize with all zeros
    for (const std::string& name : ik_solver->getJointNames())
      params.seed_state.insert(std::pair<std::string, double>(name, 0.0));
  }
  else
  {
    for (auto it = seed_state_config.begin(); it != seed_state_config.end(); ++it)
    {
      const YAML::Node& seed_state_entry = *it;
      std::string name = reach::get<std::string>(seed_state_entry, "name");
      double position = reach::get<double>(seed_state_entry, "position");
      params.seed_state.insert(std::pair<std::string, double>(name, position));
    }
    // Check that a seed state is defined for all joints known to the IK interface
    for (const std::string& name : ik_solver->getJointNames())
      if (params.seed_state.find(name) == params.seed_state.end())
        throw std::runtime_error("Seed state parameter does not include joint '" + name + "'");
  }

  // Load the target pose generator plugin
  reach::TargetPoseGenerator::ConstPtr target_pose_generator;
  {
    auto factory = loader.createInstance<TargetPoseGeneratorFactory>(get<std::string>(pose_gen_config, "name"));
    target_pose_generator = factory->create(pose_gen_config);
  }

  // Load the evaluator plugin
  reach::Evaluator::ConstPtr evaluator;
  {
    auto factory = loader.createInstance<EvaluatorFactory>(get<std::string>(eval_config, "name"));
    evaluator = factory->create(eval_config);
  }

  // Load the display plugin
  reach::Display::ConstPtr display;
  {
    auto factory = loader.createInstance<DisplayFactory>(get<std::string>(display_config, "name"));
    display = factory->create(display_config);
  }

  // Load the logger plugin
  reach::Logger::Ptr logger;
  {
    auto factory = loader.createInstance<LoggerFactory>(get<std::string>(logger_config, "name"));
    logger = factory->create(logger_config);
  }

  // Initialize the reach study
  reach::ReachStudy rs(ik_solver, evaluator, target_pose_generator, display, logger, params);

  const boost::filesystem::path db_file = results_dir / config_name / "reach.db.xml";

  if (boost::filesystem::exists(db_file))
  {
    // Attempt to load the database first, if it exists
    rs.load(db_file.string());

    if (rs.getDatabase().results.size() == 1)
    {
      logger->print("Loaded un-optimized database");
      rs.optimize();

      // Overwrite the file
      rs.save(db_file.string());
    }
    else
    {
      logger->print("Loaded optimized database");
    }
  }
  else
  {
    if (!results_dir.empty())
      boost::filesystem::create_directories(results_dir / config_name);

    // Run the reach study
    rs.run();

    // Save the preliminary results
    if (!results_dir.empty())
      rs.save(db_file.string());

    // Optimize the reach study
    rs.optimize();

    // Save the optimized results
    if (!results_dir.empty())
      rs.save(db_file.string());
  }

  // Show the results
  const reach::ReachDatabase& db = rs.getDatabase();
  logger->printResults(db.calculateResults());
  display->showEnvironment();
  display->showResults(db.results.back());

  auto handleSignal = [](int /*sig*/) { throw std::runtime_error("Reach study temrinated"); };
  signal(SIGINT, handleSignal);
  signal(SIGTERM, handleSignal);

  if (wait_after_completion)
  {
    logger->print("Press enter to quit");
    std::cin.get();
  }
}

}  // namespace reach
