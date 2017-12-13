#include <ros/ros.h>
#include <ros/package.h>
#include <robot_reach_study/reach_database.h>
#include <boost/filesystem.hpp>

const static std::string RESULTS_FOLDER_NAME = "results";
const static std::string OPT_DB_NAME = "optimized_reach.db";

bool get_all(const boost::filesystem::path& root,
             const std::string& ext,
             std::vector<std::pair<boost::filesystem::path, boost::filesystem::path>>& ret)
{
  if(!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) return false;

  boost::filesystem::recursive_directory_iterator it(root);
  boost::filesystem::recursive_directory_iterator endit;

  while(it != endit)
  {
    if(boost::filesystem::is_regular_file(*it) && it->path().extension() == ext)
    {
      // Capture only the optimized reach databases
      if(it->path().filename() == OPT_DB_NAME)
      {
        std::pair<boost::filesystem::path, boost::filesystem::path> tmp;
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

  std::string root_path = ros::package::getPath("robot_reach_study") + "/" + RESULTS_FOLDER_NAME;

  if(argv[1])
  {
    const std::string folder_name = argv[1];
    root_path += "/" + folder_name;
  }

  boost::filesystem::path root (root_path);
  std::vector<std::pair<boost::filesystem::path, boost::filesystem::path>> files;
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

    robot_reach_study::Database db;
    if(db.load(path))
    {
      std::cout << boost::format("%-30s %=25.3f %=25.6f %=25.3f %=25.3f\n")
                   % config.c_str()
                   % db.getReachPercentage()
                   % db.getNormalizedTotalPoseScore()
                   % db.getAverageNeighborsCount()
                   % db.getAverageJointDistance();
    }
  }

  return 0;
}
