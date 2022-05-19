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
#include "rclcpp/rclcpp.hpp"
#include "reach_core/reach_database.h"

#include <filesystem>

#include <boost/format.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

const static std::string RESULTS_FOLDER_NAME = "results";
const static std::string OPT_DB_NAME = "optimized_reach.db";
// const static std::string OPT_DB_NAME = "reach.db";

typedef std::pair<double, double> coordinate_pair_type;
typedef std::pair<std::filesystem::path, coordinate_pair_type> coordinate_path;
typedef std::pair<std::string, coordinate_path> coordinate_config;
typedef std::unordered_map<std::string, std::vector<coordinate_config>>
    coordinate_config_map_of_vecotors;

bool get_all(
    const std::filesystem::path &root, const std::string &ext,
    std::vector<std::pair<std::filesystem::path, std::filesystem::path>> &ret) {
  if (!std::filesystem::exists(root) || !std::filesystem::is_directory(root))
    return false;

  std::filesystem::recursive_directory_iterator it(root);
  std::filesystem::recursive_directory_iterator endit;

  while (it != endit) {
    if (std::filesystem::is_regular_file(*it) &&
        it->path().extension() == ext) {
      // Capture only the optimized reach databases
      if (it->path().filename() == OPT_DB_NAME) {
        std::pair<std::filesystem::path, std::filesystem::path> tmp;
        tmp.first = it->path().parent_path().filename();
        tmp.second = it->path();
        ret.push_back(tmp);
      }
    }
    ++it;
  }

  std::sort(
      ret.begin(), ret.end(),
      [&](const std::pair<std::filesystem::path, std::filesystem::path> &first,
          std::pair<std::filesystem::path, std::filesystem::path> &second) {
        reach::core::ReachDatabase db;
        // first
        db.load(first.second);
        reach::core::StudyResults res = db.getStudyResults();
        float first_reach_percentage = res.reach_percentage;
        // second
        db.load(second.second);
        res = db.getStudyResults();
        float second_reach_percentage = res.reach_percentage;
        return first_reach_percentage > second_reach_percentage;
      });

  return true;
}

int main(int argc, char **argv) {
  // Initialize ROS
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options(
      rclcpp::NodeOptions()
          .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true));
  // create node
  auto node = std::make_shared<rclcpp::Node>("data_loader_node", options);

  std::string pkg_name;
  std::string dir_name;
  bool chk_all_sub_dirs;
  bool avg_neighbor_count;
  bool print_result_total = false;

  node->get_parameter_or<std::string>("package_name", pkg_name, "reach_core");
  node->get_parameter_or<std::string>("directory_name", dir_name,
                                      RESULTS_FOLDER_NAME);
  node->get_parameter_or<bool>("check_all_subdirectories", chk_all_sub_dirs,
                               false);
  node->get_parameter_or<bool>("avg_neighbor_count", avg_neighbor_count, false);
  node->get_parameter_or<bool>("print_total", print_result_total, true);

  std::string root_path =
      std::string(ament_index_cpp::get_package_share_directory(pkg_name)) +
      "/" + dir_name;

  if (argv[1] && !chk_all_sub_dirs) {
    const std::string folder_name = argv[1];
    root_path += "/" + folder_name;
  }

  std::filesystem::path root(root_path);
  std::vector<std::pair<std::filesystem::path, std::filesystem::path>> files;

  if (!get_all(root, ".db", files)) {
    std::cout << "Specified directory does not exist";
    return 0;
  }

  if (print_result_total) {
    if (avg_neighbor_count) {
      std::cout << boost::format("%-60s %=25s %=25s %=25s %=25s\n") %
                       "Configuration Name" % "Reach Percentage" %
                       "Normalized Total Pose Score" %
                       "Average Reachable Neighbors" % "Average Joint Distance";
    } else {
      std::cout << boost::format("%-60s %=25s %=25s\n") % "Configuration Name" %
                       "Reach Percentage" % "Normalized Total Pose Score";
    }

    for (size_t i = 0; i < files.size(); ++i) {
      const std::string config = files[i].first.string();
      const std::string path = files[i].second.string();
      reach::core::ReachDatabase db;
      if (db.load(path)) {
        reach::core::StudyResults res = db.getStudyResults();
        if (avg_neighbor_count) {
          std::cout << boost::format(
                           "%-60s %=25.3f %=25.6f %=25.3f %=25.3f\n") %
                           config.c_str() % res.reach_percentage %
                           res.norm_total_pose_score % res.avg_num_neighbors %
                           res.avg_joint_distance;
        } else {
          std::cout << boost::format("%-60s %=25.3f %=25.6f\n") %
                           config.c_str() % res.reach_percentage %
                           res.norm_total_pose_score;
        }
      }
    }
  }

  // shutdown
  rclcpp::shutdown();

  return 0;
}
