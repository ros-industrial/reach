#include "reach/utils/database_utils.h"
#include <fstream>
#include <ros/serialization.h>
#include <moveit/robot_state/conversions.h>

namespace reach
{
namespace utils
{

template <class T>
bool toFile(const std::string& path,
            const T& msg)
{
  namespace ser = ros::serialization;
  uint32_t serialize_size = ser::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serialize_size]);

  ser::OStream stream(buffer.get(), serialize_size);
  ser::serialize(stream, msg);

  std::ofstream file(path.c_str(), std::ios::out | std::ios::binary);
  if (!file)
  {
    return false;
  }
  else
  {
    file.write((char*)buffer.get(), serialize_size);
    return file.good();
  }
}

template <class T>
bool fromFile(const std::string& path,
              T& msg)
{
  namespace ser = ros::serialization;

  std::ifstream ifs(path.c_str(), std::ios::in | std::ios::binary);
  if (!ifs)
  {
    return false;
  }

  ifs.seekg(0, std::ios::end);
  std::streampos end = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  std::streampos begin = ifs.tellg();

  uint32_t file_size = end - begin;

  boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
  ifs.read((char*)ibuffer.get(), file_size);
  ser::IStream istream(ibuffer.get(), file_size);
  ser::deserialize(istream, msg);
  return true;
}

robot_reach_study::ReachRecord makeRecordSuccess(const std::string &id,
                                                 const geometry_msgs::Pose &goal,
                                                 const moveit::core::RobotState &seed_state,
                                                 const moveit::core::RobotState &goal_state,
                                                 const double score)
{
  robot_reach_study::ReachRecord r;
  r.id = id;
  r.goal = goal;
  r.reached = true;
  moveit::core::robotStateToRobotStateMsg(seed_state, r.seed_state);
  moveit::core::robotStateToRobotStateMsg(goal_state, r.goal_state);
  r.score = score;
  return r;
}

robot_reach_study::ReachRecord makeRecordFailure(const std::string &id,
                                                 const geometry_msgs::Pose &goal,
                                                 const moveit::core::RobotState &seed_state,
                                                 const double score)
{
  robot_reach_study::ReachRecord r;
  r.id = id;
  r.goal = goal;
  r.reached = false;
  moveit::core::robotStateToRobotStateMsg(seed_state, r.seed_state);
  moveit::core::robotStateToRobotStateMsg(seed_state, r.goal_state);
  r.score = score;
  return r;
}

} // namespace utils
} // namespace reach

