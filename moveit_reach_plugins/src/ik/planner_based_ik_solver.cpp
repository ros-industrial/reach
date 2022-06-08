/*
 * Copyright 2022 PickNik, Inc.
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
/* Authors: Lovro Ivanov, @livanov93
   Desc:
*/

#include "moveit_reach_plugins/ik/planner_based_ik_solver.h"

#include "moveit_reach_plugins/utils.h"
#include "tf2_eigen/tf2_eigen.h"

#include <algorithm>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <reach_core/utils/general_utils.h>

namespace {

template <typename T>
T clamp(const T& val, const T& low, const T& high) {
  return std::max(low, std::min(val, high));
}

}  // namespace

namespace moveit_reach_plugins {
namespace ik {

PlannerBasedIKSolver::PlannerBasedIKSolver() : MoveItIKSolver() {}

bool PlannerBasedIKSolver::initialize(
    std::string& name, rclcpp::Node::SharedPtr node,
    const std::shared_ptr<const moveit::core::RobotModel> model) {
  // initialize base class
  if (!MoveItIKSolver::initialize(name, node, model)) {
    RCLCPP_ERROR(LOGGER, "Failed to initialize PlannerBasedIKSolver plugin");
    return false;
  }

  // get parameters for cartesian path computation
  try {
    if (!node->get_parameter("ik_solver_config.retrieval_distance",
                             retrieval_path_length_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.retrieval_distance' ");
      return false;
    }
    if (!node->get_parameter("ik_solver_config.max_eef_step", max_eef_step_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.max_eef_step' ");
      return false;
    }
    if (!node->get_parameter("ik_solver_config.jump_threshold",
                             jump_threshold_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.jump_threshold' ");
      return false;
    }
    if (!node->get_parameter("ik_solver_config.tool_frame", tool_frame_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.jump_threshold' ");
      return false;
    }
    // make sure it is positive to follow solvers logic
    retrieval_path_length_ = std::abs(double(retrieval_path_length_));
    max_eef_step_ = std::abs(double(max_eef_step_));
    jump_threshold_ = std::abs(double(jump_threshold_));

    if (!planner_) {
      PlanningSpecs spec;
      spec._robot_model = model;
      if (!node->get_parameter("ik_solver_config.pipeline_name",
                               pipeline_name_)) {
        RCLCPP_ERROR(LOGGER,
                     "No parameter defined by the name "
                     "'ik_solver_config.pipeline_name' ");
        return false;
      }

      spec._planning_pipeline = pipeline_name_;
      planner_ = create(node_, spec);
    } else if (model != planner_->getRobotModel()) {
      throw std::runtime_error(
          "Difference in robot models between planner and ik solver");
    }

    if (!node->get_parameter("ik_solver_config.display_motion_plans",
                             display_motion_plans_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.display_motion_plans' ");
      return false;
    }
    planner_->displayComputedMotionPlans(display_motion_plans_);

    if (!node->get_parameter("ik_solver_config.publish_planning_requests",
                             publish_planning_requests_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.publish_planning_requests' ");
      return false;
    }
    planner_->publishReceivedRequests(publish_planning_requests_);

    if (!node->get_parameter("ik_solver_config.num_planning_attempts",
                             num_planning_attempts_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.num_planning_attempts' ");
      return false;
    }

    if (!node->get_parameter("ik_solver_config.max_velocity_scaling_factor",
                             max_velocity_scaling_factor_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.max_velocity_scaling_factor' ");
      return false;
    }

    if (!node->get_parameter("ik_solver_config.max_acceleration_scaling_factor",
                             max_acceleration_scaling_factor_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.max_acceleration_scaling_factor' ");
      return false;
    }

    // TODO(livanov93) see if this can go to get_parameter_or with default blank
    // string
    if (!node->get_parameter("ik_solver_config.planner_id", planner_id_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.planner_id' ");
      return false;
    }
    if (!node->get_parameter("ik_solver_config.allowed_planning_time",
                             allowed_planning_time_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.allowed_planning_time' ");
      return false;
    }
    if (!node->get_parameter("ik_solver_config.poses_frame_id",
                             poses_frame_id_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'ik_solver_config.poses_frame_id' ");
      return false;
    }

    node->get_parameter_or("ik_solver_config.goal_joint_tolerance",
                           goal_joint_tolerance_, 1e-4);
    node->get_parameter_or("ik_solver_config.goal_position_tolerance",
                           goal_position_tolerance_, 1e-4);
    node->get_parameter_or("ik_solver_config.goal_orientation_tolerance",
                           goal_orientation_tolerance_, 1e-4);

    workspace_parameter_ = moveit_msgs::msg::WorkspaceParameters();

    // additional planning scene for planner
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(model_);

    // Check that the input collision mesh frame exists
    if (!planning_scene_->knowsFrameTransform(collision_mesh_frame_)) {
      RCLCPP_ERROR_STREAM(LOGGER, "Specified collision mesh frame '"
                                      << collision_mesh_frame_
                                      << "' does not exist");
      return false;
    }

    // Add the collision object to the planning scene
    const std::string object_name = "reach_object";
    moveit_msgs::msg::CollisionObject obj = utils::createCollisionObject(
        collision_mesh_package_, collision_mesh_frame_, object_name);
    if (!planning_scene_->processCollisionObjectMsg(obj)) {
      RCLCPP_ERROR(LOGGER, "Failed to add collision mesh to planning scene");
      return false;
    } else {
      planning_scene_->getAllowedCollisionMatrixNonConst().setEntry(
          object_name, touch_links_, true);
    }

  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(LOGGER, ex.what());
    return false;
  }

  robot_start_state_pub_ =
      node->create_publisher<moveit_msgs::msg::DisplayRobotState>(
          "planner_based_ik_solver_start_state", 1);

  // output message about successful initialization
  RCLCPP_INFO(LOGGER, "Successfully initialized PlannerBasedIKSolver plugin");
  return true;
}

// https://github.com/ros-planning/moveit_task_constructor/blob/60229db010ea305296bc1c90d04faa3e4dacd976/core/src/solvers/pipeline_planner.cpp#L51-L75
struct PlannerCache {
  using PlannerID = std::tuple<std::string, std::string>;
  using PlannerMap =
      std::map<PlannerID, std::weak_ptr<planning_pipeline::PlanningPipeline>>;
  using ModelList = std::list<
      std::pair<std::weak_ptr<const moveit::core::RobotModel>, PlannerMap>>;
  ModelList cache_;

  PlannerMap::mapped_type& retrieve(
      const moveit::core::RobotModelConstPtr& model, const PlannerID& id) {
    // find model in cache_ and remove expired entries while doing so
    ModelList::iterator model_it = cache_.begin();
    while (model_it != cache_.end()) {
      if (model_it->first.expired()) {
        model_it = cache_.erase(model_it);
        continue;
      }
      if (model_it->first.lock() == model) break;
      ++model_it;
    }
    if (model_it ==
        cache_.end())  // if not found, create a new PlannerMap for this model
      model_it =
          cache_.insert(cache_.begin(), std::make_pair(model, PlannerMap()));

    return model_it->second
        .insert(std::make_pair(id, PlannerMap::mapped_type()))
        .first->second;
  }
};

// https://github.com/ros-planning/moveit_task_constructor/blob/60229db010ea305296bc1c90d04faa3e4dacd976/core/src/solvers/pipeline_planner.cpp#L77-L102
planning_pipeline::PlanningPipelinePtr PlannerBasedIKSolver::create(
    const rclcpp::Node::SharedPtr& node,
    const PlannerBasedIKSolver::PlanningSpecs& spec) {
  static PlannerCache cache;

  static constexpr char const* PLUGIN_PARAMETER_NAME = "planning_plugin";

  std::string pipeline_ns = spec._namespace;
  const std::string parameter_name = pipeline_ns + "." + PLUGIN_PARAMETER_NAME;
  // fallback to old structure for pipeline parameters in MoveIt
  if (!node->has_parameter(parameter_name)) {
    node->declare_parameter(parameter_name,
                            rclcpp::ParameterType::PARAMETER_STRING);
  }
  if (std::string parameter; !node->get_parameter(parameter_name, parameter)) {
    RCLCPP_WARN(node->get_logger(), "Failed to find '%s.%s'. %s",
                pipeline_ns.c_str(), PLUGIN_PARAMETER_NAME,
                "Attempting to load pipeline from old parameter structure. "
                "Please update your MoveIt config.");
    pipeline_ns = "move_group";
  }

  PlannerCache::PlannerID id(pipeline_ns, spec._planning_adapter_param);

  std::weak_ptr<planning_pipeline::PlanningPipeline>& entry =
      cache.retrieve(spec._robot_model, id);
  planning_pipeline::PlanningPipelinePtr planner = entry.lock();
  if (!planner) {
    // create new entry
    planner = std::make_shared<planning_pipeline::PlanningPipeline>(
        spec._robot_model, node, pipeline_ns, PLUGIN_PARAMETER_NAME,
        spec._planning_adapter_param);
    // store in cache
    entry = planner;
  }
  return planner;
}

std::optional<double> PlannerBasedIKSolver::solveIKFromSeed(
    const Eigen::Isometry3d& target, const std::map<std::string, double>& seed,
    std::vector<double>& solution, std::vector<double>& joint_space_trajectory,
    std::vector<double>& cartesian_space_waypoints, double& fraction) {
  moveit::core::RobotState seed_state(model_);

  const std::vector<std::string>& joint_names =
      jmg_->getActiveJointModelNames();

  std::vector<double> seed_subset;
  if (!utils::transcribeInputMap(seed, joint_names, seed_subset)) {
    RCLCPP_ERROR_STREAM(
        LOGGER, __FUNCTION__ << ": failed to transcribe input pose map");
    return {};
  }

  seed_state.setJointGroupPositions(jmg_, seed_subset);
  seed_state.update();

  // initialize motion plan request
  moveit_msgs::msg::MotionPlanRequest req;
  req.group_name = jmg_->getName();
  req.planner_id = planner_id_;
  req.allowed_planning_time = allowed_planning_time_;

  moveit::core::robotStateToRobotStateMsg(seed_state, req.start_state);

  // publish robot start state
  moveit_msgs::msg::DisplayRobotState robot_start_state_msg;
  robot_start_state_msg.state = req.start_state;
  robot_start_state_pub_->publish(robot_start_state_msg);

  req.num_planning_attempts = num_planning_attempts_;
  req.max_velocity_scaling_factor = max_velocity_scaling_factor_;
  req.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;
  req.workspace_parameters = workspace_parameter_;

  req.goal_constraints.resize(1);
  geometry_msgs::msg::PoseStamped pose_to_plan;
  pose_to_plan.header.frame_id = poses_frame_id_;
  pose_to_plan.pose = tf2::toMsg(target);
  req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
      tool_frame_, pose_to_plan, goal_position_tolerance_,
      goal_orientation_tolerance_);

  moveit_msgs::msg::PlanningScene ps;
  ::planning_interface::MotionPlanResponse res;

  bool success = planner_->generatePlan(planning_scene_, req, res);

  if (success) {
    moveit::core::RobotState state(res.trajectory_->getLastWayPoint());
    solution.clear();
    state.copyJointGroupPositions(jmg_, solution);

    // Convert back to map
    std::map<std::string, double> solution_map;
    for (std::size_t i = 0; i < solution.size(); ++i) {
      solution_map.emplace(joint_names[i], solution[i]);
    }

    fraction = 1.0;

    if (fraction != 0.0) {
      const size_t trajectory_size = res.trajectory_->size();
      const size_t joints_size = joint_names.size();
      size_t joint_space_idx = 0;
      size_t cartesian_space_idx = 0;
      const size_t joint_space_traj_size = trajectory_size * joints_size;
      const size_t cartesian_space_wpts_size = trajectory_size * 7;
      joint_space_trajectory.resize(joint_space_traj_size);
      cartesian_space_waypoints.resize(cartesian_space_wpts_size);

      for (size_t i = 0;
           i < trajectory_size && joint_space_idx < joint_space_traj_size &&
           cartesian_space_idx < cartesian_space_wpts_size;
           ++i) {
        std::vector<double> joint_positions_tmp;
        res.trajectory_->getWayPoint(i).copyJointGroupPositions(
            jmg_, joint_positions_tmp);
        geometry_msgs::msg::Pose pose_tmp = tf2::toMsg(
            res.trajectory_->getWayPoint(i).getFrameTransform(tool_frame_));

        for (size_t j = 0; j < joints_size; ++j) {
          joint_space_trajectory[joint_space_idx + j] = joint_positions_tmp[j];
        }
        joint_space_idx += joints_size;

        cartesian_space_waypoints[cartesian_space_idx + 0] =
            pose_tmp.position.x;
        cartesian_space_waypoints[cartesian_space_idx + 1] =
            pose_tmp.position.y;
        cartesian_space_waypoints[cartesian_space_idx + 2] =
            pose_tmp.position.z;
        cartesian_space_waypoints[cartesian_space_idx + 3] =
            pose_tmp.orientation.x;
        cartesian_space_waypoints[cartesian_space_idx + 4] =
            pose_tmp.orientation.y;
        cartesian_space_waypoints[cartesian_space_idx + 5] =
            pose_tmp.orientation.z;
        cartesian_space_waypoints[cartesian_space_idx + 6] =
            pose_tmp.orientation.w;
        cartesian_space_idx += 7;
      }

      return fraction * eval_->calculateScore(solution_map);
    } else {
      // make sure trajectory is empty on exit
      cartesian_space_waypoints.clear();
      joint_space_trajectory.clear();
      return {};
    }
  } else {
    fraction = 0.0;
    return {};
  }
}

}  // namespace ik
}  // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::ik::PlannerBasedIKSolver,
                       reach::plugins::IKSolverBase)
