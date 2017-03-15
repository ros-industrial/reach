#ifndef PARAM_HELPERS_H
#define PARAM_HELPERS_H

#include <ros/serialization.h>
#include <fstream>

namespace robot_reach_study
{
// Write a message to disk
template <class T> inline bool toFile(const std::string& path, const T& msg)
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

// Restore a message from disk
template <class T> inline bool fromFile(const std::string& path, T& msg)
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

} // end namespace

#endif // PARAM_HELPERS_H
