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
#include "reach_core/reach_database.h"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


#include <filesystem>
#include <boost/format.hpp>

const static std::string RESULTS_FOLDER_NAME = "results";
const static std::string OPT_DB_NAME = "optimized_reach.db";

bool get_all(const std::filesystem::path& root,
             const std::string& ext,
             std::vector<std::pair<std::filesystem::path, std::filesystem::path>>& ret)
{
  if(!std::filesystem::exists(root) || !std::filesystem::is_directory(root)) return false;

  std::filesystem::recursive_directory_iterator it(root);
  std::filesystem::recursive_directory_iterator endit;

  while(it != endit)
  {
    if(std::filesystem::is_regular_file(*it) && it->path().extension() == ext)
    {
      // Capture only the optimized reach databases
      if(it->path().filename() == OPT_DB_NAME)
      {
        std::pair<std::filesystem::path, std::filesystem::path> tmp;
        tmp.first = it->path().parent_path().filename();
        tmp.second = it->path();
        ret.push_back(tmp);
      }
    }
    ++it;
  }

  std::sort(ret.begin(), ret.end());

  return true;
}

int main(int argc, char **argv)
{
  if(argc > 2)
  {
    return -1;
  }

    // Initialize ROS
    rclcpp::init(argc, argv);
    // create node
    auto node = std::make_shared<rclcpp::Node>("data_loader_node");


  std::string root_path = std::string(ament_index_cpp::get_package_share_directory(("reach_core"))) + "/" + RESULTS_FOLDER_NAME;

  if(argv[1])
  {
    const std::string folder_name = argv[1];
    root_path += "/" + folder_name;
  }

  std::filesystem::path root (root_path);
  std::vector<std::pair<std::filesystem::path, std::filesystem::path>> files;
  if(!get_all(root, ".db", files))
  {
    std::cout << "Specified directory does not exist";
    return 0;
  }

  std::cout << boost::format("%-30s %=25s %=25s %=25s %=25s\n")
               % "Configuration Name"
               % "Reach Percentage"
               % "Normalized Total Pose Score"
               % "Average Reachable Neighbors"
               % "Average Joint Distance";

  for(size_t i = 0; i < files.size(); ++i)
  {
    const std::string config = files[i].first.string();
    const std::string path = files[i].second.string();

    reach::core::ReachDatabase db;
    if(db.load(path))
    {
      reach::core::StudyResults res = db.getStudyResults();
      std::cout << boost::format("%-30s %=25.3f %=25.6f %=25.3f %=25.3f\n")
                   % config.c_str()
                   % res.reach_percentage
                   % res.norm_total_pose_score
                   % res.avg_num_neighbors
                   % res.avg_joint_distance;
    }
  }
    // shutdown
    rclcpp::shutdown();

  return 0;
}
