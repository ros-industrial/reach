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
#ifndef REACH_UTILS_DATABASE_UTILS_H
#define REACH_UTILS_DATABASE_UTILS_H

#include <fstream>
#include <moveit/robot_state/robot_state.h>
#include <reach_msgs/msg/reach_record.h>
#include <rclcpp/serialization.hpp
#include <string>

namespace reach
{
  namespace utils
  {

    template <class T>
    bool toFile(const std::string &path,
                const T &msg)
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
        file.write((char *)buffer.get(), serialize_size);
        return file.good();
      }
    }

    template <class T>
    bool fromFile(const std::string &path,
                  T &msg)
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
      ifs.read((char *)ibuffer.get(), file_size);
      ser::IStream istream(ibuffer.get(), file_size);
      ser::deserialize(istream, msg);
      return true;
    }

  } // namespace utils
} // namespace reach

#endif // REACH_UTILS_DATABASE_UTILS_H
