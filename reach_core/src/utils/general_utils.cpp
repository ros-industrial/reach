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
#include "reach_core/utils/general_utils.h"

#include <rclcpp/rclcpp.hpp>

namespace reach {
namespace {
const rclcpp::Logger LOGGER = rclcpp::get_logger("reach.general_utils");
}
namespace utils {

void integerProgressPrinter(std::atomic<int> &current_counter,
                            std::atomic<int> &previous_pct,
                            const int total_size) {
  const float current_pct_float = (static_cast<float>(current_counter.load()) /
                                   static_cast<float>(total_size)) *
                                  100.0;
  const int current_pct = static_cast<int>(current_pct_float);
  if (current_pct > previous_pct.load()) {
    RCLCPP_INFO(LOGGER, "[%d%%]", current_pct);
  }
  previous_pct = current_pct;
}

Eigen::Isometry3d createFrame(const Eigen::Vector3f &pt,
                              const Eigen::Vector3f &norm) {
  // Initialize coordinate frame and set XYZ location
  Eigen::Isometry3f p = Eigen::Isometry3f::Identity();
  p.matrix()(0, 3) = pt(0);
  p.matrix()(1, 3) = pt(1);
  p.matrix()(2, 3) = pt(2);

  // Create plane from point normal
  Eigen::Hyperplane<float, 3> plane(norm, Eigen::Vector3f(0, 0, 0));

  // If the normal and global x-axis are not closely aligned
  if (std::abs(norm.dot(Eigen::Vector3f::UnitX())) < 0.90) {
    // Project the global x-axis onto the plane to generate the x-axis
    Eigen::Vector3f x_axis =
        plane.projection(Eigen::Vector3f::UnitX()).normalized();
    p.matrix().col(0).head<3>() = x_axis;
    p.matrix().col(1).head<3>() = norm.cross(x_axis);
    p.matrix().col(2).head<3>() = norm;
  } else {
    // Project the global y-axis onto the plane to generate the y-axis
    Eigen::Vector3f y_axis =
        plane.projection(Eigen::Vector3f::UnitY()).normalized();
    p.matrix().col(0).head<3>() = y_axis.cross(norm);
    p.matrix().col(1).head<3>() = y_axis;
    p.matrix().col(2).head<3>() = norm;
  }

  return p.cast<double>();
}

void trajectoryFiller(
    const std::vector<std::vector<double>> &trajectory,
    const std::vector<Eigen::Isometry3d> &waypoints,
    const std::vector<std::string> &names,
    std::vector<geometry_msgs::msg::Pose> &cartesian_space_waypoints,
    std::vector<sensor_msgs::msg::JointState> &joint_space_trajectory) {
  //  const size_t trajectory_size = trajectory.size();
  //  cartesian_space_waypoints.resize(trajectory_size);
  //  joint_space_trajectory.resize(trajectory_size);

  //  for (size_t k = 0; k < trajectory_size; ++k) {
  //    geometry_msgs::msg::Pose csw_tmp;
  //    csw_tmp.position.set__x(waypoints[k].translation().x());
  //    csw_tmp.position.set__y(waypoints[k].translation().y());
  //    csw_tmp.position.set__z(waypoints[k].translation().z());
  //    Eigen::Quaterniond q(waypoints[k].rotation());
  //    csw_tmp.orientation.set__x(q.x());
  //    csw_tmp.orientation.set__y(q.y());
  //    csw_tmp.orientation.set__z(q.z());
  //    csw_tmp.orientation.set__w(q.w());
  //    cartesian_space_waypoints[k] = csw_tmp;
  //
  //    sensor_msgs::msg::JointState jst_tmp;
  //    jst_tmp.name = names;
  //    jst_tmp.position = trajectory[k];
  //    joint_space_trajectory[k] = jst_tmp;
  //  }
}

}  // namespace utils
}  // namespace reach
