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
#include "reach_core/reach_study.h"
#include "reach_core/study_parameters.h"

#include <memory>


class RobotReachStudyNode : public reach::core::ReachStudy
{
public:
   explicit RobotReachStudyNode(std::string& node_name) :
    reach::core::ReachStudy(node_name,
        rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
    {
        // get the study parameters
        getStudyParameters();
    }

public:
    bool getStudyParameters(){

        // fetch parameteres !this->get_parameter("config_name", sp_.config_name)  ||
        if (!this->get_parameter("fixed_frame", sp_.fixed_frame) ||
            !this->get_parameter("results_package", sp_.results_package) ||
            !this->get_parameter("results_directory", sp_.results_directory) ||
            !this->get_parameter("object_frame", sp_.object_frame) ||
            !this->get_parameter("pcd_package", sp_.pcd_package) ||
            !this->get_parameter("pcd_filename_path", sp_.pcd_filename_path) ||
            !this->get_parameter("optimization.radius", sp_.optimization.radius) ||
            !this->get_parameter("optimization.max_steps", sp_.optimization.max_steps) ||
            !this->get_parameter("optimization.step_improvement_threshold", sp_.optimization.step_improvement_threshold) ||
            !this->get_parameter("get_avg_neighbor_count", sp_.get_neighbors) ||
            !this->get_parameter("compare_dbs", sp_.compare_dbs) ||
            !this->get_parameter("visualize_results", sp_.visualize_results) ||
            !this->get_parameter("ik_solver_config.name", sp_.ik_solver_config_name) ||
            !this->get_parameter("display_config.name", sp_.display_config_name) ) {
            RCLCPP_ERROR(rclcpp::get_logger("robot_reach_study_node"), "One of the main parameters do not exist..." );
            return false;
        }else{
            RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"), "fixed_frame: '%s'", sp_.fixed_frame.c_str() );
            RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"), "results_package: '%s'", sp_.results_package.c_str() );
            RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"), "results_directory: '%s'", sp_.results_directory.c_str() );
            RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"), "object_frame: '%s'", sp_.object_frame.c_str() );
            RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"), "pcd_package: '%s'", sp_.pcd_package.c_str() );
            RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"), "pcd_filename: '%s'", sp_.pcd_filename_path.c_str() );
            RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"), "optimization.radius: '%f'", sp_.optimization.radius );
            RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"), "optimization.max_steps: '%d'", sp_.optimization.max_steps );
            RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"), "optimization.step_improvement_threshold: '%f'", sp_.optimization.step_improvement_threshold );
            RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"), "get_avg_neighbor_count: '%d'", sp_.get_neighbors );
            RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"), "compare_dbs: '%s'", sp_.compare_dbs[0].c_str() );
            RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"), "visualize_results: '%c'", sp_.visualize_results );
            RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"), "ik_solver_config.name: '%s'", sp_.ik_solver_config_name.c_str() );
            RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"), "display_config.name: '%s'", sp_.display_config_name.c_str() );
            return true;
        }
    }

    bool run(){

        return this->run(sp_);
    }

private:

    reach::core::StudyParameters sp_;

};



int main(int argc, char **argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    // create node
    auto node = std::make_shared<RobotReachStudyNode>("robot_reach_study_node");


  // Run the reach study
//  if(!node->run())
//  {
//    RCLCPP_ERROR(rclcpp::get_logger("robot_reach_study_node"), "Unable to perform the reach study");
//    return -1;
//  }

    // spin
    rclcpp::spin(node);


  return 0;
}
