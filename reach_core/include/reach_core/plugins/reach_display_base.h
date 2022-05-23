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

#include "reach_core/utils/visualization_utils.h"

#include <rclcpp/rclcpp.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <reach_msgs/msg/reach_database.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// PoseStamped
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_model/robot_model.h>

constexpr char INTERACTIVE_MARKER_TOPIC[] = "reach_int_markers";
constexpr char REACH_DIFF_TOPIC[] = "reach_comparison";
constexpr char MARKER_TOPIC[] = "reach_neighbors";
constexpr char POSE_TOPIC[] = "reach_pose";

namespace reach {
namespace {
const rclcpp::Logger LOGGER =
    rclcpp::get_logger("moveit_servo.reach_display_base");
}

namespace plugins {

class DisplayBase {
 public:
  DisplayBase() = default;

  virtual ~DisplayBase() {
    server_.reset();
    diff_pub_.reset();
    marker_pub_.reset();
    db_visualize_pub_.reset();
  }

  virtual bool initialize(
      std::string &name, rclcpp::Node::SharedPtr node,
      const std::shared_ptr<const moveit::core::RobotModel> model) {
    node_ = node;
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        INTERACTIVE_MARKER_TOPIC, node);
    diff_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        REACH_DIFF_TOPIC, 1);
    marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
        MARKER_TOPIC, 1);

    db_visualize_pub_ =
        node->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualized_dbs", 1);

    RCLCPP_INFO(LOGGER, "Initialized DisplayBase plugin!");
    return true;
  };

  virtual void showEnvironment() = 0;

  virtual void showEnvironment(const std::vector<std::string> &names,
                               const std::vector<double> &positions) = 0;

  virtual void updateRobotPose(const std::map<std::string, double> &pose) = 0;

  virtual void updateRobotTrajectory(
      const std::vector<std::map<std::string, double>> &trajectory) = 0;

  void addInteractiveMarkerData(
      const reach_msgs::msg::ReachDatabase &database) {
    server_->clear();
    for (const reach_msgs::msg::ReachRecord &rec : database.records) {
      auto marker =
          utils::makeInteractiveMarker(node_, rec, fixed_frame_, marker_scale_);
      server_->insert(std::move(marker));
      menu_handler_.apply(*server_, rec.id);
    }
    server_->applyChanges();
  }

  void createMenuFunction(
      const std::string &menu_entry,
      const interactive_markers::MenuHandler::FeedbackCallback &callback) {
    menu_handler_.insert(menu_entry, callback);
  }

  void updateInteractiveMarker(const reach_msgs::msg::ReachRecord &rec) {
    if (server_->erase(rec.id)) {
      auto marker =
          utils::makeInteractiveMarker(node_, rec, fixed_frame_, marker_scale_);
      server_->insert(marker);
      menu_handler_.apply(*server_, rec.id);
      server_->applyChanges();
    } else {
      RCLCPP_ERROR_STREAM(LOGGER, "Failed to update interactive marker '"
                                      << rec.id
                                      << "'; marker does not alreadys exist");
    }
  }

  void publishMarkerArray(const std::vector<std::string> &ids) {
    if (!ids.empty()) {
      std::vector<geometry_msgs::msg::Point> pt_array;

      for (const std::string &id : ids) {
        visualization_msgs::msg::InteractiveMarker marker;
        if (!server_->get(id, marker)) {
          RCLCPP_ERROR_STREAM(LOGGER, "Failed to get interactive marker '"
                                          << id << "' from server");
          return;
        }

        // Visualize points reached around input point
        pt_array.push_back(marker.pose.position);
      }

      // Create points marker, publish it, and move robot to result state for
      // given point
      visualization_msgs::msg::Marker pt_marker = reach::utils::makeMarker(
          node_, pt_array, fixed_frame_, marker_scale_);
      marker_pub_->publish(pt_marker);
    }
  }

  void compareDatabases(
      const std::map<std::string, reach_msgs::msg::ReachDatabase> &data) {
    const std::size_t n_perm = pow(2, data.size());
    const std::size_t n_records = data.begin()->second.records.size();

    // Check that all databases are the same size
    for (auto it = std::next(data.begin()); it != data.end(); ++it) {
      if (it->second.records.size() != n_records) {
        RCLCPP_FATAL(LOGGER, "Mismatched database sizes");
      }
    }

    // Generate a list of all possible namespace permutations
    std::vector<std::string> ns_vec(n_perm);
    ns_vec[0] = "not_all";
    ns_vec[n_perm - 1] = "all";

    for (char perm_ind = 1; perm_ind < static_cast<char>(n_perm - 1);
         ++perm_ind) {
      std::string ns_name("");
      for (auto it = data.begin(); it != data.end(); ++it) {
        if (((perm_ind >> std::distance(data.begin(), it)) & 1) == 1) {
          if (ns_name == "") {
            ns_name += it->first;
          } else {
            ns_name += "_AND_" + it->first;
          }
        }
      }
      ns_vec[static_cast<std::size_t>(perm_ind)] = ns_name;
    }

    // Set the color of the arrow display
    std::vector<float> color(4);
    color[0] = 1.0;
    color[1] = 0.0;
    color[2] = 0.0;
    color[3] = 1.0;
    boost::optional<std::vector<float>> arrow_color = color;

    // Create Rviz marker array
    visualization_msgs::msg::MarkerArray marker_array;

    // Iterate over all records in the databases and compare whether or not they
    // were reached in that database
    for (std::size_t i = 0; i < n_records; ++i) {
      // Create a binary code based on whether the point was reached
      // code LSB is msg.reach boolean of 1st database
      // code << n is is msg.reach boolean of (n+1)th database
      char code = 0;

      for (auto it = data.begin(); it != data.end(); ++it) {
        code += static_cast<char>(it->second.records[i].reached)
                << std::distance(data.begin(), it);
      }

      // Create Rviz marker unless the point was reached by all or none of the
      // robot configurations
      if (code != 0 && code != n_perm - 1) {
        std::string ns = {ns_vec[static_cast<std::size_t>(code)]};
        visualization_msgs::msg::Marker arrow_marker =
            utils::makeVisual(node_, data.begin()->second.records[i],
                              fixed_frame_, marker_scale_, ns, {arrow_color});
        marker_array.markers.push_back(arrow_marker);
      }
    }

    diff_pub_->publish(marker_array);
  }

  void visualizeDatabases(
      const std::map<std::string, reach_msgs::msg::ReachDatabase> &data) {
    const std::size_t n_records = data.begin()->second.records.size();

    // Check that all databases are the same size
    for (auto it = std::next(data.begin()); it != data.end(); ++it) {
      if (it->second.records.size() != n_records) {
        RCLCPP_FATAL(LOGGER, "Mismatched database sizes");
      }
    }

    // Create Rviz marker array
    visualization_msgs::msg::MarkerArray marker_array;

    // Iterate over all records in the databases and compare whether or not they
    // were reached in that database
    RCLCPP_INFO(LOGGER, "Visualizing databases with '%zu' records.", n_records);
    for (std::size_t i = 0; i < n_records; ++i) {
      visualization_msgs::msg::Marker arrow_marker;
      for (auto it = data.begin(); it != data.end(); ++it) {
        arrow_marker =
            utils::makeVisual(node_, it->second.records[i], fixed_frame_,
                              marker_scale_, it->first);
        if (it->second.records[i].reached) {
          marker_array.markers.push_back(arrow_marker);
        }
      }
    }

    db_visualize_pub_->publish(marker_array);
  }

 public:
  std::shared_ptr<rclcpp::Node> node_;

 protected:
  std::string fixed_frame_ = "base";

  double marker_scale_ = 1.0;

 private:
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  interactive_markers::MenuHandler menu_handler_;

  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>>
      diff_pub_;

  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>>
      marker_pub_;
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>>
      db_visualize_pub_;
};
typedef std::shared_ptr<DisplayBase> DisplayBasePtr;

}  // namespace plugins
}  // namespace reach

#endif  // REACH_CORE_PLUGINS_DISPLAY_DISPLAY_BASE_H
