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
#ifndef REACH_CORE_PLUGINS_DISPLAY_DISPLAY_BASE_H
#define REACH_CORE_PLUGINS_DISPLAY_DISPLAY_BASE_H

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <reach_msgs/ReachDatabase.h>
#include "reach_core/utils/visualization_utils.h"
#include <visualization_msgs/MarkerArray.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <pcl/point_types_conversion.h>

const static std::string INTERACTIVE_MARKER_TOPIC = "reach_int_markers";
const static std::string REACH_DIFF_TOPIC = "reach_comparison";
const static std::string MARKER_TOPIC = "reach_neighbors";

namespace reach
{
namespace plugins
{
class DisplayBase
{
public:
  DisplayBase() : server_(INTERACTIVE_MARKER_TOPIC)
  {
    diff_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(REACH_DIFF_TOPIC, 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(MARKER_TOPIC, 1, true);
  }

  virtual ~DisplayBase()
  {
  }

  virtual bool initialize(XmlRpc::XmlRpcValue& config) = 0;

  virtual void showEnvironment() = 0;

  virtual void updateRobotPose(const std::map<std::string, double>& pose) = 0;

  void addInteractiveMarkerData(const reach_msgs::ReachDatabase& database)
  {
    server_.clear();

    // Find the max of the scores
    double max_score = 0;
    for (const reach_msgs::ReachRecord& rec : database.records)
    {
      if (rec.score > max_score)
      {
        max_score = rec.score;
      }
    }

    for (const reach_msgs::ReachRecord& rec : database.records)
    {
      static float max_h = 0.75f * 360.0f;  // Corresponds to blue color
      float h = max_h - (static_cast<float>(rec.score / max_score) * max_h);
      float s = 1.0f;
      float v = rec.reached ? 1.0f : 0.0f;

      // Convert to RGB
      pcl::PointXYZHSV pt_hsv(h, s, v);
      pcl::PointXYZRGB pt_rgb;
      pcl::PointXYZHSVtoXYZRGB(pt_hsv, pt_rgb);
      Eigen::Vector3f rgb_color = pt_rgb.getRGBVector3i().cast<float>() / 255.0f;

      auto marker = utils::makeInteractiveMarker(rec, fixed_frame_, marker_scale_, rgb_color);
      server_.insert(std::move(marker));
      menu_handler_.apply(server_, rec.id);
    }
    server_.applyChanges();
  }

  void createMenuFunction(const std::string& menu_entry,
                          const interactive_markers::MenuHandler::FeedbackCallback& callback)
  {
    menu_handler_.insert(menu_entry, callback);
  }

  void updateInteractiveMarker(const reach_msgs::ReachRecord& rec)
  {
    if (server_.erase(rec.id))
    {
      auto marker = utils::makeInteractiveMarker(rec, fixed_frame_, marker_scale_);
      server_.insert(marker);
      menu_handler_.apply(server_, rec.id);
      server_.applyChanges();
    }
    else
    {
      ROS_ERROR_STREAM("Failed to update interactive marker '" << rec.id << "'; marker does not alreadys exist");
    }
  }

  void publishMarkerArray(const std::vector<std::string>& ids)
  {
    if (!ids.empty())
    {
      std::vector<geometry_msgs::Point> pt_array;

      for (const std::string& id : ids)
      {
        visualization_msgs::InteractiveMarker marker;
        if (!server_.get(id, marker))
        {
          ROS_ERROR_STREAM("Failed to get interactive marker '" << id << "' from server");
          return;
        }

        // Visualize points reached around input point
        pt_array.push_back(marker.pose.position);
      }

      // Create points marker, publish it, and move robot to result state for  given point
      visualization_msgs::Marker pt_marker = utils::makeMarker(pt_array, fixed_frame_, marker_scale_);
      marker_pub_.publish(pt_marker);
    }
  }

  void compareDatabases(const std::map<std::string, reach_msgs::ReachDatabase>& data)
  {
    const std::size_t n_perm = pow(2, data.size());
    const std::size_t n_records = data.begin()->second.records.size();

    // Check that all databases are the same size
    for (auto it = std::next(data.begin()); it != data.end(); ++it)
    {
      if (it->second.records.size() != n_records)
      {
        ROS_FATAL("Mismatched database sizes");
      }
    }

    // Generate a list of all possible namespace permutations
    std::vector<std::string> ns_vec(n_perm);
    ns_vec[0] = "not_all";
    ns_vec[n_perm - 1] = "all";

    for (char perm_ind = 1; perm_ind < static_cast<char>(n_perm - 1); ++perm_ind)
    {
      std::string ns_name = "";
      for (auto it = data.begin(); it != data.end(); ++it)
      {
        if (((perm_ind >> std::distance(data.begin(), it)) & 1) == 1)
        {
          if (ns_name == "")
          {
            ns_name += it->first;
          }
          else
          {
            ns_name += "_AND_" + it->first;
          }
        }
      }
      ns_vec[static_cast<std::size_t>(perm_ind)] = ns_name;
    }

    // Create Rviz marker array
    visualization_msgs::MarkerArray marker_array;

    // Iterate over all records in the databases and compare whether or not they were reached in that database
    for (std::size_t i = 0; i < n_records; ++i)
    {
      // Create a binary code based on whether the point was reached
      // code LSB is msg.reach boolean of 1st database
      // code << n is is msg.reach boolean of (n+1)th database
      char code = 0;

      for (auto it = data.begin(); it != data.end(); ++it)
      {
        code += static_cast<char>(it->second.records[i].reached) << std::distance(data.begin(), it);
      }

      // Create Rviz marker unless the point was reached by all or none of the robot configurations
      if (code != 0 && code != n_perm - 1)
      {
        std::string ns = { ns_vec[static_cast<std::size_t>(code)] };
        visualization_msgs::Marker arrow_marker =
            utils::makeVisual(data.begin()->second.records[i], fixed_frame_, marker_scale_, ns);
        marker_array.markers.push_back(arrow_marker);
      }
    }

    diff_pub_.publish(marker_array);
  }

protected:
  std::string fixed_frame_ = "base_frame";

  double marker_scale_ = 1.0;

private:
  interactive_markers::InteractiveMarkerServer server_;

  interactive_markers::MenuHandler menu_handler_;

  ros::NodeHandle nh_;

  ros::Publisher diff_pub_;

  ros::Publisher marker_pub_;
};
typedef boost::shared_ptr<DisplayBase> DisplayBasePtr;

}  // namespace plugins
}  // namespace reach

#endif  // REACH_CORE_PLUGINS_DISPLAY_DISPLAY_BASE_H
