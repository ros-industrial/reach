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

#include <ros/ros.h>
#include <pluginlib/class_loader.h>

static const std::string PACKAGE = "reach_core";
static const std::string IK_BASE_CLASS = "reach::IKSolverBase";
static const std::string DISPLAY_BASE_CLASS = "reach::DisplayBase";
static const std::string TARGET_POSE_GENERATOR_BASE_CLASS = "reach::WaypointGeneratorBase";

template <typename T>
T get(const ros::NodeHandle& nh, const std::string& key)
{
  T val;
  if (!nh.getParam(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "robot_reach_study_node");
    ros::NodeHandle pnh("~"), nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Get the study parameters
    reach::ReachStudy::Parameters params;
    params.radius = get<double>(nh, "optimization/radius");
    params.max_steps = get<int>(nh, "optimization/max_steps");
    params.step_improvement_threshold = get<double>(nh, "optimization/step_improvement_threshold");

    // Load the IK Solver plugin
    pluginlib::ClassLoader<reach::IKSolver> solver_loader(PACKAGE, IK_BASE_CLASS);
    reach::IKSolver::Ptr ik_solver;
    {
      XmlRpc::XmlRpcValue config = get<XmlRpc::XmlRpcValue>(nh, "ik_solver_config");
      ik_solver = solver_loader.createInstance(config["name"]);
      ik_solver->initialize(config);
    }

    // Load the target pose generator plugin
    pluginlib::ClassLoader<reach::TargetPoseGenerator> target_pose_generator_loader_(
        PACKAGE, TARGET_POSE_GENERATOR_BASE_CLASS);
    reach::TargetPoseGenerator::Ptr target_pose_generator;
    {
      XmlRpc::XmlRpcValue config = get<XmlRpc::XmlRpcValue>(nh, "target_pose_generator_config");
      target_pose_generator = target_pose_generator_loader_.createInstance(config["name"]);
      target_pose_generator->initialize(config);
    }

    const std::string config_name = get<std::string>(nh, config_name);
    boost::filesystem::path results_dir(get<std::string>(nh, "results_directory"));

    // Initialize the reach study
    reach::ReachStudy rs(ik_solver, target_pose_generator, params, config_name);

    // Run the reach study
    rs.run();
    rs.save((results_dir / "study.db").string());
    rs.optimize();
    rs.save((results_dir / "study_optimized.db").string());
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
  }

  return 0;
}
