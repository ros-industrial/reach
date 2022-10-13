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

#include <boost/filesystem.hpp>
#include <numeric>
#include <thread>

namespace reach
{
ReachStudy::ReachStudy(IKSolver::ConstPtr ik_solver, Evaluator::ConstPtr evaluator, TargetPoseGenerator::ConstPtr target_generator,
                       const Parameters params, const std::string& name)
  : params_(std::move(params))
  , db_(new ReachDatabase(name))
  , ik_solver_(std::move(ik_solver))
  , evaluator_(std::move(evaluator))
  , target_poses_(target_generator->generate())
{
}

void ReachStudy::load(const std::string& filename)
{
  *db_ = reach::load(filename);
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
  // First loop through all points in point cloud and get IK solution
  std::atomic<int> current_counter, previous_pct;
  current_counter = previous_pct = 0;

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
    integerProgressPrinter(current_counter, previous_pct, target_poses_.size());
  }
}

void ReachStudy::optimize()
{
  // Create an efficient search tree for doing nearest neighbors search
  {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (auto it = db_->begin(); it != db_->end(); ++it)
    {
      pcl::PointXYZ pt;
      pt.getVector3fMap() = it->second.goal.translation().cast<float>();
      cloud->push_back(pt);
    }
    search_tree_ = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    search_tree_->setInputCloud(cloud);
  }

  // Create sequential vector to be randomized
  std::vector<std::size_t> rand_vec(db_->size());
  std::iota(rand_vec.begin(), rand_vec.end(), 0);

  // Iterate
  std::atomic<int> current_counter, previous_pct;
  int n_opt = 0;
  float previous_score = 0.0;
  float pct_improve = 1.0;

  while (pct_improve > params_.step_improvement_threshold && n_opt < params_.max_steps)
  {
    std::cout << "Entering optimization loop " << n_opt << std::endl;
    previous_score = db_->calculateResults().norm_total_pose_score;
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
      integerProgressPrinter(current_counter, previous_pct, target_poses_.size());
    }

    // Recalculate optimized reach study results
    auto results = db_->calculateResults();
    std::cout << results.print() << std::endl;
    pct_improve = std::abs((results.norm_total_pose_score - previous_score) / previous_score);
    ++n_opt;
  }
}

std::tuple<double, double> ReachStudy::getAverageNeighborsCount() const
{
  std::cout << "--------------------------------------------" << std::endl;
  std::cout << "Beginning average neighbor count calculation" << std::endl;

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
      reachNeighborsRecursive(db_, msg, ik_solver_, evaluator_, params_.radius, result, search_tree_);

      neighbor_count += static_cast<int>(result.reached_pts.size() - 1);
      total_joint_distance = total_joint_distance + result.joint_distance;
    }

    // Print function progress
    ++current_counter;
    integerProgressPrinter(current_counter, previous_pct, total);
  }

  float avg_neighbor_count = static_cast<float>(neighbor_count.load()) / static_cast<float>(db_->size());
  float avg_joint_distance =
      static_cast<float>(total_joint_distance.load()) / static_cast<float>(neighbor_count.load());

  return std::make_tuple(avg_neighbor_count, avg_joint_distance);
}

}  // namespace reach
