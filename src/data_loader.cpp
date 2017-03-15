#include <ros/ros.h>
#include <ros/package.h>
#include <robot_reach_study/reach_database.h>

int main(int argc, char **argv)
{
  std::vector<std::string> configs;

  configs.push_back("abb_rail_bmw");
  configs.push_back("abb_rail_chrysler");
  configs.push_back("abb_rail_fiat");
  configs.push_back("abb_rail_ford");
  configs.push_back("abb_rail_gmc");
  configs.push_back("abb_rail_mercedes");
  configs.push_back("abb_rail_ram");

  configs.push_back("motoman_rail_bmw");
  configs.push_back("motoman_rail_chrysler");
  configs.push_back("motoman_rail_fiat");
  configs.push_back("motoman_rail_ford");
  configs.push_back("motoman_rail_gmc");
  configs.push_back("motoman_rail_mercedes");
  configs.push_back("motoman_rail_ram");

  configs.push_back("kawasaki_rail_bmw");
  configs.push_back("kawasaki_rail_chrysler");
  configs.push_back("kawasaki_rail_fiat");
  configs.push_back("kawasaki_rail_ford");
  configs.push_back("kawasaki_rail_gmc");
  configs.push_back("kawasaki_rail_mercedes");
  configs.push_back("kawasaki_rail_ram");

  configs.push_back("abb_inverted_bmw");
  configs.push_back("abb_inverted_chrysler");
  configs.push_back("abb_inverted_fiat");
  configs.push_back("abb_inverted_ford");
  configs.push_back("abb_inverted_gmc");
  configs.push_back("abb_inverted_mercedes");
  configs.push_back("abb_inverted_ram");

  configs.push_back("motoman_inverted_bmw");
  configs.push_back("motoman_inverted_chrysler");
  configs.push_back("motoman_inverted_fiat");
  configs.push_back("motoman_inverted_ford");
  configs.push_back("motoman_inverted_gmc");
  configs.push_back("motoman_inverted_mercedes");
  configs.push_back("motoman_inverted_ram");

  configs.push_back("kawasaki_inverted_bmw");
  configs.push_back("kawasaki_inverted_chrysler");
  configs.push_back("kawasaki_inverted_fiat");
  configs.push_back("kawasaki_inverted_ford");
  configs.push_back("kawasaki_inverted_gmc");
  configs.push_back("kawasaki_inverted_mercedes");
  configs.push_back("kawasaki_inverted_ram");

  configs.push_back("motoman_sideslung_bmw");
  configs.push_back("motoman_sideslung_chrysler");
  configs.push_back("motoman_sideslung_fiat");
  configs.push_back("motoman_sideslung_ford");
  configs.push_back("motoman_sideslung_gmc");
  configs.push_back("motoman_sideslung_mercedes");
  configs.push_back("motoman_sideslung_ram");

  configs.push_back("fanuc_rail_bmw");
  configs.push_back("fanuc_rail_chrysler");
  configs.push_back("fanuc_rail_fiat");
  configs.push_back("fanuc_rail_ford");
  configs.push_back("fanuc_rail_gmc");
  configs.push_back("fanuc_rail_mercedes");
  configs.push_back("fanuc_rail_ram");

  configs.push_back("fanuc_inverted_bmw");
  configs.push_back("fanuc_inverted_chrysler");
  configs.push_back("fanuc_inverted_fiat");
  configs.push_back("fanuc_inverted_ford");
  configs.push_back("fanuc_inverted_gmc");
  configs.push_back("fanuc_inverted_mercedes");
  configs.push_back("fanuc_inverted_ram");

  const std::string pkg_path = ros::package::getPath("robot_reach_study");

  std::cout << boost::format("%-30s %=25s %=25s\n") % "Configuration Name" % "Reach Percentage" % "Normalized Total Score";
  for(size_t i = 0; i < configs.size(); ++i)
  {
    const std::string config = configs[i];
    const std::string path = pkg_path + "/output/" + config + "/optimized_reach.db";
    robot_reach_study::Database db;
    if(db.load(path))
    {
      std::cout << boost::format("%-30s %=25.2f %=25.6f\n") % config.c_str() % db.getReachPercentage() % db.getNormalizedTotalScore();
    }
  }

  return 0;
}
