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
// const static std::string OPT_DB_NAME = "optimized_reach.db";
const static std::string OPT_DB_NAME = "reach.db";

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
  bool print_per_patient_config = false;
  bool print_result_total = false;
  bool print_per_coordinate = false;

  node->get_parameter_or<std::string>("package_name", pkg_name, "reach_core");
  node->get_parameter_or<std::string>("directory_name", dir_name,
                                      RESULTS_FOLDER_NAME);
  node->get_parameter_or<bool>("check_all_subdirectories", chk_all_sub_dirs,
                               false);
  node->get_parameter_or<bool>("avg_neighbor_count", avg_neighbor_count, false);
  node->get_parameter_or<bool>("print_total", print_result_total, false);
  node->get_parameter_or<bool>("print_per_patient_config",
                               print_per_patient_config, false);
  node->get_parameter_or<bool>("print_per_coordinate", print_per_coordinate,
                               false);

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

  // specific printing
  if (print_per_patient_config) {
    std::map<
        std::string,
        std::vector<std::pair<
            std::string, std::pair<std::string, reach::core::StudyResults>>>>
        tmp_storage;
    for (size_t i = 0; i < files.size(); ++i) {
      const std::string config = files[i].first.string();
      const std::string path = files[i].second.string();
      size_t idx_start = config.find('(') + 1;
      size_t idx_end = config.find(')') - 1;
      std::string config_name_map = config.substr(0, idx_start - 2);

      reach::core::ReachDatabase db;
      if (db.load(path)) {
        reach::core::StudyResults res = db.getStudyResults();
        tmp_storage[config_name_map].push_back(
            std::make_pair(path, std::make_pair(config, res)));
      }
    }

    for (auto it = tmp_storage.begin(); it != tmp_storage.end(); it++) {
      std::sort(it->second.begin(), it->second.end(),
                [&](std::pair<std::string,
                              std::pair<std::string, reach::core::StudyResults>>
                        &first,
                    std::pair<std::string,
                              std::pair<std::string, reach::core::StudyResults>>
                        &second) {
                  reach::core::ReachDatabase db;
                  // first
                  db.load(first.first);
                  reach::core::StudyResults res = db.getStudyResults();
                  float first_reach_percentage = res.reach_percentage;
                  // second
                  db.load(second.first);
                  res = db.getStudyResults();
                  float second_reach_percentage = res.reach_percentage;
                  return first_reach_percentage > second_reach_percentage;
                });
    }

    for (auto it = tmp_storage.begin(); it != tmp_storage.end(); it++) {
      std::cout << boost::format(
                       "-------------------------------------------------------"
                       "---------------------\n")
                << std::endl;

      std::cout << boost::format("%-60s %=25s %=25s\n") % "Configuration Name" %
                       "Reach Percentage" % "Normalized Total Pose Score";
      std::cout << boost::format(
                       "-------------------------------------------------------"
                       "---------------------\n")
                << std::endl;

      std::cout << boost::format(
                       "%-60s "
                       "#######################################################"
                       "###################\n") %
                       it->first;
      for (auto &iter : it->second) {
        std::cout << boost::format("%-60s %=25.3f %=25.6f\n") %
                         iter.second.first %
                         iter.second.second.reach_percentage %
                         iter.second.second.norm_total_pose_score;
      }
    }
  }

  if (print_per_coordinate) {
    std::map<
        std::string,
        std::vector<std::pair<
            std::string, std::pair<std::string, reach::core::StudyResults>>>>
        tmp_storage;
    for (size_t i = 0; i < files.size(); ++i) {
      const std::string config = files[i].first.string();
      const std::string path = files[i].second.string();
      size_t idx_start = config.find('(') + 1;
      size_t idx_end = config.find(')') - 1;
      std::string config_name_map = config.substr(idx_start, idx_end);

      reach::core::ReachDatabase db;
      if (db.load(path)) {
        reach::core::StudyResults res = db.getStudyResults();
        tmp_storage[config_name_map].push_back(
            std::make_pair(path, std::make_pair(config, res)));
      }
    }

    std::map<std::string, double> per_coordinate_percentage;
    for (auto it = tmp_storage.begin(); it != tmp_storage.end(); it++) {
      per_coordinate_percentage[it->first] = 0.0;
      for (auto &item : it->second) {
        per_coordinate_percentage[it->first] +=
            item.second.second.reach_percentage;
      }
      per_coordinate_percentage[it->first] /= it->second.size();
    }
    std::vector<std::pair<std::string, double>> per_coordinate_percenage_vec(
        per_coordinate_percentage.begin(), per_coordinate_percentage.end());

    std::sort(per_coordinate_percenage_vec.begin(),
              per_coordinate_percenage_vec.end(),
              [&](const std::pair<std::string, double> &first,
                  std::pair<std::string, double> &second) {
                return first.second > second.second;
              });

    std::cout << boost::format(
                     "---------------------------------------------------------"
                     "-------------------\n")
              << std::endl;

    std::cout << boost::format("%-60s %=25s\n") % "Configuration Name" %
                     "Reach Percentage";
    std::cout << boost::format(
                     "---------------------------------------------------------"
                     "-------------------\n")
              << std::endl;

    for (auto it = per_coordinate_percenage_vec.begin();
         it != per_coordinate_percenage_vec.end(); it++) {
      std::cout << boost::format("%-60s %=25.3f\n") % it->first % it->second;
    }
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
