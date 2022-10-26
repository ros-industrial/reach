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
#include <reach_core/utils.h>
#include <reach_core/plugin_utils.h>

#include <boost/filesystem.hpp>
#include <boost_plugin_loader/plugin_loader.hpp>
#include <numeric>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace reach
{
ReachStudy::ReachStudy(IKSolver::ConstPtr ik_solver, Evaluator::ConstPtr evaluator,
                       TargetPoseGenerator::ConstPtr target_generator, Display::ConstPtr display,
                       Logger::ConstPtr logger, const Parameters params, const std::string& name)
  : params_(std::move(params))
  , db_(new ReachDatabase(name))
  , ik_solver_(std::move(ik_solver))
  , evaluator_(std::move(evaluator))
  , display_(std::move(display))
  , logger_(std::move(logger))
  , target_poses_(target_generator->generate())
{
}

void ReachStudy::load(const std::string& filename)
{
  *db_ = reach::load(filename);
  search_tree_ = createSearchTree(*db_);
  display_->showEnvironment();
  display_->showResults(*db_);
}

void ReachStudy::save(const std::string& filename) const
{
  reach::save(*db_, filename);
}

ReachDatabase::ConstPtr ReachStudy::getDatabase() const
{
  return db_;
}

void ReachStudy::run()
{
  logger_->print("Starting reach study");
  logger_->setMaxProgress(target_poses_.size());

  // Show display
  display_->showEnvironment();
  display_->showResults(*db_);

  // First loop through all points in point cloud and get IK solution
  std::atomic<unsigned long> current_counter;
  current_counter = 0;

#pragma omp parallel for num_threads(std::thread::hardware_concurrency())
  for (std::size_t i = 0; i < target_poses_.size(); ++i)
  {
    const Eigen::Isometry3d& tgt_frame = target_poses_[i];

    // Get the seed position
    const std::vector<std::string> joint_names = ik_solver_->getJointNames();
    std::map<std::string, double> seed_state = zip(joint_names, std::vector<double>(joint_names.size(), 0.0));

    // Solve IK
    try
    {
      std::vector<double> solution;
      double score;
      std::tie(solution, score) = evaluateIK(tgt_frame, seed_state, ik_solver_, evaluator_);

      ReachRecord msg(std::to_string(i), true, tgt_frame, seed_state, zip(ik_solver_->getJointNames(), solution), score);
      db_->put(msg);
    }
    catch(const std::exception&)
    {
      ReachRecord msg(std::to_string(i), false, tgt_frame, seed_state, seed_state, 0.0);
      db_->put(msg);
    }

    // Print function progress
    current_counter++;
    logger_->printProgress(current_counter.load());
  }

  logger_->print("Reach study complete");
  display_->showResults(*db_);
}

void ReachStudy::optimize()
{
  logger_->print("Starting optimization");

  // Show environment display
  display_->showEnvironment();
  display_->showResults(*db_);

  // Create an efficient search tree for doing nearest neighbors search
  search_tree_ = createSearchTree(*db_);

  // Create sequential vector to be randomized
  std::vector<std::size_t> rand_vec(db_->size());
  std::iota(rand_vec.begin(), rand_vec.end(), 0);

  // Iterate
  std::atomic<unsigned long> current_counter;
  int n_opt = 0;
  float previous_score = 0.0;
  float pct_improve = 1.0;

  while (pct_improve > params_.step_improvement_threshold && n_opt < params_.max_steps)
  {
    logger_->print("Entering optimization loop " + std::to_string(n_opt));
    logger_->setMaxProgress(target_poses_.size());

    previous_score = db_->calculateResults().norm_total_pose_score;
    current_counter = 0;

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
        std::vector<ReachRecord> result =
            reachNeighborsDirect(db_, msg, ik_solver_, evaluator_, params_.radius, search_tree_);

        // Replace the old records if the scores of the new records are higher
        for (const ReachRecord& rec : result)
        {
          const ReachRecord& old_rec = db_->get(rec.id);
          if (rec.score > old_rec.score)
          {
            db_->put(rec);
          }
        }
      }

      // Print function progress
      current_counter++;
      logger_->printProgress(current_counter.load());
    }

    // Recalculate optimized reach study results
    auto results = db_->calculateResults();
    logger_->printResults(results);

    pct_improve = std::abs((results.norm_total_pose_score - previous_score) / previous_score);
    ++n_opt;

    // Show the results
    display_->showResults(*db_);
  }

  logger_->print("Optimization complete");
}

std::tuple<double, double> ReachStudy::getAverageNeighborsCount() const
{
  logger_->print("Beginning average neighbor count calculation");

  std::atomic<unsigned long> current_counter, neighbor_count;
  current_counter = neighbor_count = 0;
  std::atomic<double> total_joint_distance;

// Iterate
#pragma parallel for num_threads(std::thread::hardware_concurrency())
  for (auto it = db_->begin(); it != db_->end(); ++it)
  {
    ReachRecord msg = it->second;
    if (msg.reached)
    {
      NeighborReachResult result;
      reachNeighborsRecursive(db_, msg, ik_solver_, evaluator_, params_.radius, result, search_tree_);

      neighbor_count += static_cast<int>(result.reached_pts.size() - 1);
      total_joint_distance = total_joint_distance + result.joint_distance;
    }

    // Print function progress
    ++current_counter;
    logger_->printProgress(current_counter.load());
  }

  float avg_neighbor_count = static_cast<float>(neighbor_count.load()) / static_cast<float>(db_->size());
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

  boost_plugin_loader::PluginLoader loader;
  loader.search_libraries_env = SEARCH_LIBRARIES_ENV;

  // Load the IK Solver plugin
  reach::IKSolver::ConstPtr ik_solver;
  {
    auto factory = loader.createInstance<IKSolverFactory>(get<std::string>(ik_config, "name"));
    ik_solver = factory->create(ik_config);
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
  reach::Logger::ConstPtr logger;
  {
    auto factory = loader.createInstance<LoggerFactory>(get<std::string>(logger_config, "name"));
    logger = factory->create(logger_config);
  }

  // Initialize the reach study
  reach::ReachStudy rs(ik_solver, evaluator, target_pose_generator, display, logger, params, config_name);

  const boost::filesystem::path db_file = results_dir / config_name / "study.db";
  const boost::filesystem::path opt_db_file = results_dir / config_name / "study_optimized.db";

  if (boost::filesystem::exists(opt_db_file))
  {
    // Attempt to load the optimized database first, if it exists
    rs.load(opt_db_file.string());
    logger->print("Loaded optimized database");
  }
  else if (boost::filesystem::exists(db_file))
  {
    // Then try to load the un-optimized database, if it exists
    rs.load(db_file.string());

    logger->print("Loaded un-optimized database");
    rs.optimize();
    rs.save((results_dir / config_name / "study_optimized.db").string());
  }
  else
  {
    if (!results_dir.empty())
      boost::filesystem::create_directories(results_dir / config_name);

    // Run the reach study
    rs.run();

    // Save the preliminary results
    if (!results_dir.empty())
      rs.save((results_dir / config_name / "study.db").string());

    // Optimize the reach study
    rs.optimize();

    // Save the optimized results
    if (!results_dir.empty())
      rs.save((results_dir / config_name / "study_optimized.db").string());
  }

  // Show the results
  reach::ReachDatabase::ConstPtr db = rs.getDatabase();
  logger->printResults(db->calculateResults());
  display->showEnvironment();
  display->showResults(*db);

  if (wait_after_completion)
  {
    logger->print("Press enter to quit");
    std::cin.get();
  }
}

}  // namespace reach
