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
#include "moveit_reach_plugins/ik/discretized_moveit_ik_solver.h"
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <algorithm>

namespace
{

template<typename T>
T clamp(const T& val,
        const T& low,
        const T& high)
{
  return std::max(low, std::min(val, high));
}

} // namespace anonymous

namespace moveit_reach_plugins
{
namespace ik
{

DiscretizedMoveItIKSolver::DiscretizedMoveItIKSolver()
  : MoveItIKSolver()
{

}

bool DiscretizedMoveItIKSolver::initialize(std::string& name, rclcpp::Node::SharedPtr &node)
{
  if(!MoveItIKSolver::initialize(name, node))
  {
    ROS_ERROR("Failed to initialize MoveItIKSolver plugin");
    return false;
  }

  try
  {
    dt_ = std::abs(double(config["discretization_angle"]));
    double clamped_dt = clamp<double>(dt_, 0.0, M_PI);
    if(std::abs(dt_ - clamped_dt) > 1.0e-6)
    {
      ROS_WARN_STREAM("Clamping discretization angle between 0 and pi; new value is " << clamped_dt);
    }
    dt_ = clamped_dt;
  }
  catch(const XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR_STREAM(ex.getMessage());
    return false;
  }

  ROS_INFO_STREAM("Successfully initialized DiscretizedMoveItIKSolver plugin");
  return true;
}

boost::optional<double> DiscretizedMoveItIKSolver::solveIKFromSeed(const Eigen::Isometry3d& target,
                                                                   const std::map<std::string, double>& seed,
                                                                   std::vector<double>& solution)
{
  // Calculate the number of discretizations necessary to achieve discretization angle
  const static int n_discretizations = int((2.0*M_PI) / dt_);

  // Set up containers for the best solution to be saved into the database
  std::vector<double> best_solution;
  double best_score = 0;

  for(int i = 0; i < n_discretizations; ++i)
  {
    Eigen::Isometry3d discretized_target (target * Eigen::AngleAxisd (double(i)*dt_, Eigen::Vector3d::UnitZ()));
    std::vector<double> tmp_solution;

    std::optional<double> score = MoveItIKSolver::solveIKFromSeed(discretized_target, seed, tmp_solution);
    if(score && (score.get() > best_score))
    {
      best_score = *score;
      best_solution = std::move(tmp_solution);
    }
    else
    {
      continue;
    }
  }

  if(best_score > 0)
  {
    solution = std::move(best_solution);
    return boost::optional<double>(best_score);
  }
  else
  {
    return {};
  }
}

} // namespace ik
} // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::ik::DiscretizedMoveItIKSolver, reach::plugins::IKSolverBase)
