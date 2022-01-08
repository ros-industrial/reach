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
#include <rclcpp/serialization.hpp>
#include <string>
#include <rcpputils/asserts.hpp>

namespace reach
{
  namespace utils
  {

    template <class T>
    bool toFile(const std::string &path,
                const T &msg)
    {
      auto serializer = rclcpp::Serialization<T>();
      rclcpp::SerializedMessage ser_msg;
      serializer.serialize_message(&msg, &ser_msg);

      std::ofstream file(path.c_str(), std::ios::out | std::ios::binary);
      if (!file)
      {
        return false;
      }
      else
      {
        file.write((char *)ser_msg.get_rcl_serialized_message().buffer, ser_msg.capacity());
        return file.good();
      }
    }

    template <class T>
    bool fromFile(const std::string &path,
                  T &msg)
    {
//      RCLCPP_INFO(rclcpp::get_logger("serialization_utils"), "Serializing from file...");
      std::ifstream ifs(path.c_str(), std::ios::in | std::ios::binary);
      if (!ifs)
      {
        RCLCPP_INFO(rclcpp::get_logger("serialization_utils"), "Stream '%s' does not exist!", path.c_str());
        return false;
      }

      ifs.seekg(0, std::ios::end);
      std::streampos end = ifs.tellg();
      ifs.seekg(0, std::ios::beg);
      std::streampos begin = ifs.tellg();

      uint32_t file_size = end - begin;

      std::shared_ptr<uint8_t[]> ibuffer(new uint8_t[file_size]);
      ifs.read((char *)ibuffer.get(), file_size);

//      for(size_t i=0; i<file_size; ++i){
//          RCLCPP_INFO(rclcpp::get_logger("serialization_utils"), " %c ", ibuffer.get()[i]);
//      }
      auto ser_msg = new rclcpp::SerializedMessage();
      ser_msg->get_rcl_serialized_message().buffer = ibuffer.get();
      ser_msg->get_rcl_serialized_message().buffer_length = file_size;
      ser_msg->get_rcl_serialized_message().buffer_capacity = file_size * sizeof (uint8_t);
      auto serializer = rclcpp::Serialization<T>();
      serializer.deserialize_message(ser_msg, &msg);
//      RCLCPP_INFO(rclcpp::get_logger("serialization_utils"), "Successfully serialized from file!");
      return true;
    }

  } // namespace utils
} // namespace reach

#endif // REACH_UTILS_DATABASE_UTILS_H
