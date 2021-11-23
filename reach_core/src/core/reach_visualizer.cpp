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
#include <rclcpp/rclcpp.hpp>
#include <reach_core/reach_visualizer.h>
#include <reach_core/utils/visualization_utils.h>
#include <reach_msgs/msg/reach_record.hpp>
#include <tf2_eigen/tf2_eigen.h>

namespace reach
{
  namespace core
  {
    namespace
    {
      const rclcpp::Logger LOGGER = rclcpp::get_logger("reach_core.reach_visualizer");
    }

    ReachVisualizer::ReachVisualizer(ReachDatabasePtr db,
                                     reach::plugins::IKSolverBasePtr solver,
                                     reach::plugins::DisplayBasePtr display,
                                     const double neighbor_radius,
                                     SearchTreePtr search_tree)
        : db_(db), solver_(solver), display_(display), search_tree_(search_tree), neighbor_radius_(neighbor_radius)
    {
      // Create menu functions for the display and tie them to members of this class
      using CBType = interactive_markers::MenuHandler::FeedbackCallback;
      using FBType = visualization_msgs::msg::InteractiveMarkerFeedbackConstPtr;

      CBType show_result_cb = boost::bind(&ReachVisualizer::showResultCB, this, _1);
      CBType show_seed_cb = boost::bind(&ReachVisualizer::showSeedCB, this, _1);
      CBType re_solve_ik_cb = boost::bind(&ReachVisualizer::reSolveIKCB, this, _1);
      CBType neighbors_direct_cb = boost::bind(&ReachVisualizer::reachNeighborsDirectCB, this, _1);
      CBType neighbors_recursive_cb = boost::bind(&ReachVisualizer::reachNeighborsRecursiveCB, this, _1);

      display_->createMenuFunction("Show Result", show_result_cb);
      display_->createMenuFunction("Show Seed Position", show_seed_cb);
      display_->createMenuFunction("Re-solve IK", re_solve_ik_cb);
      display_->createMenuFunction("Show Reach to Neighbors (Direct)", neighbors_direct_cb);
      display_->createMenuFunction("Show Reach to Neighbors (Recursive)", neighbors_recursive_cb);

      // Add interactive markers to the display from the reach database
      display_->addInteractiveMarkerData(db_->toReachDatabaseMsg());
    }

    void ReachVisualizer::update()
    {
      display_->addInteractiveMarkerData(db_->toReachDatabaseMsg());
    }

    void ReachVisualizer::reSolveIKCB(const visualization_msgs::msg::InteractiveMarkerFeedback *&fb)
    {
      boost::optional<reach_msgs::msg::ReachRecord> lookup = db_->get(fb->marker_name);
      if (lookup)
      {
        const std::vector<double> &seed_pose = lookup->seed_state.position;
        const std::vector<std::string> &joint_names = lookup->seed_state.name;
        std::map<std::string, double> seed_map;
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
          seed_map.emplace(joint_names[i], seed_pose[i]);
        }

        Eigen::Isometry3d target;
        tf::poseMsgToEigen(lookup->goal, target);

        // Re-solve IK at the selected marker
        std::vector<double> goal_pose;
        boost::optional<double> score = solver_->solveIKFromSeed(target, seed_map, goal_pose);

        // Update the database if the IK solution was valid
        if (score)
        {
          RCLCPP_INFO(LOGGER, "Solution found for point");

          lookup->reached = true;
          lookup->score = *score;
          lookup->goal_state.position = goal_pose;

          // Update the interactive marker server
          display_->updateInteractiveMarker(*lookup);
          display_->updateRobotPose(jointStateMsgToMap(lookup->goal_state));

          // Update the database
          db_->put(*lookup);
        }
        else
        {
          RCLCPP_INFO(LOGGER, "No solution found for point");
        }
      }
      else
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Record '" << fb->marker_name << "' does not exist in the reach database");
      }
    }

    void ReachVisualizer::showResultCB(const visualization_msgs::msg::InteractiveMarkerFeedback *&fb)
    {
      auto lookup = db_->get(fb->marker_name);
      if (lookup)
      {
        display_->updateRobotPose(jointStateMsgToMap(lookup->goal_state));
      }
      else
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Record '" << fb->marker_name << "' does not exist in the reach database");
      }
    }

    void ReachVisualizer::showSeedCB(const visualization_msgs::msg::InteractiveMarkerFeedback *&fb)
    {
      auto lookup = db_->get(fb->marker_name);
      if (lookup)
      {
        display_->updateRobotPose(jointStateMsgToMap(lookup->seed_state));
      }
      else
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Record '" << fb->marker_name << "' does not exist in the reach database");
      }
    }

    void ReachVisualizer::reachNeighborsDirectCB(const visualization_msgs::msg::InteractiveMarkerFeedback *&fb)
    {
      auto lookup = db_->get(fb->marker_name);
      if (lookup)
      {
        NeighborReachResult result = reachNeighborsDirect(db_, *lookup, solver_, neighbor_radius_, search_tree_);

        display_->updateRobotPose(jointStateMsgToMap(lookup->goal_state));
        display_->publishMarkerArray(result.reached_pts);

        RCLCPP_INFO(LOGGER, "%lu points are reachable from this pose", result.reached_pts.size());
        showResultCB(fb);
      }
      else
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Record '" << fb->marker_name << "' does not exist in the reach database");
      }
    }

    void ReachVisualizer::reachNeighborsRecursiveCB(const visualization_msgs::msg::InteractiveMarkerFeedback *&fb)
    {
      auto lookup = db_->get(fb->marker_name);
      if (lookup)
      {
        NeighborReachResult result;
        reachNeighborsRecursive(db_, *lookup, solver_, neighbor_radius_, result, search_tree_);

        display_->updateRobotPose(jointStateMsgToMap(lookup->goal_state));
        display_->publishMarkerArray(result.reached_pts);
        RCLCPP_INFO(LOGGER, "%lu points are reachable from this pose", result.reached_pts.size());
        RCLCPP_INFO(LOGGER, "Total joint distance to all neighbors: %f", result.joint_distance);
        showResultCB(fb);
      }
      else
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Record '" << fb->marker_name << "' does not exist in the reach database");
      }
    }

  } // namespace core
} // namespace reach
