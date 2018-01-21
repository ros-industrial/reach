#ifndef REACH_UTILS_DATABASE_UTILS_H
#define REACH_UTILS_DATABASE_UTILS_H

#include <fstream>
#include <moveit/robot_state/robot_state.h>
#include <reach_msgs/ReachRecord.h>
#include <ros/serialization.h>
#include <string>

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

/**
 * @brief makeRecord creates a ReachRecord message for the solution of the robot's IK for a given target pose
 * @param id
 * @param reached
 * @param goal
 * @param seed_state
 * @param goal_state
 * @param score
 * @return a ReachRecord message containing information about the robot pose, to be saved in the reach database
 */
reach_msgs::ReachRecord makeRecord(const std::string& id,
                                   const bool reached,
                                   const geometry_msgs::Pose& goal,
                                   const moveit::core::RobotState& seed_state,
                                   const moveit::core::RobotState& goal_state,
                                   const double score);

} // namespace utils
} // namespace reach

#endif // REACH_UTILS_DATABASE_UTILS_H
