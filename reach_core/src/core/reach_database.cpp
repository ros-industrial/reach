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
#include <rclcpp/rclcpp.hpp>

#include <reach_core/reach_database.h>
#include <reach_core/utils/serialization_utils.h>

namespace {

reach_msgs::msg::ReachDatabase toReachDatabase(
    const std::unordered_map<std::string, reach_msgs::msg::ReachRecord> &map,
    const reach::core::StudyResults &results) {
  reach_msgs::msg::ReachDatabase msg;
  for (auto it = map.begin(); it != map.end(); ++it) {
    msg.records.push_back(it->second);
  }

  msg.total_pose_score = results.total_pose_score;
  msg.norm_total_pose_score = results.norm_total_pose_score;
  msg.reach_percentage = results.reach_percentage;
  msg.avg_num_neighbors = results.avg_num_neighbors;
  msg.avg_joint_distance = results.avg_joint_distance;

  return msg;
}

}  // namespace

namespace reach {
namespace core {
namespace {
const rclcpp::Logger LOGGER = rclcpp::get_logger("reach_core.reach_database");
}

reach_msgs::msg::ReachRecord makeRecord(
    const std::string &id, const bool reached,
    const geometry_msgs::msg::Pose &goal,
    const sensor_msgs::msg::JointState &seed_state,
    const sensor_msgs::msg::JointState &goal_state, const double score,
    const std::string &ik_solver_name, const std::vector<double> &waypoints,
    const std::vector<double> &trajectory, double retrieved_fraction) {
  reach_msgs::msg::ReachRecord r;
  r.id = id;
  r.goal = goal;
  r.reached = reached;
  r.seed_state = seed_state;
  r.goal_state = goal_state;
  r.score = score;
  r.retrieved_fraction = retrieved_fraction;
  r.ik_solver = ik_solver_name;
  r.joint_space_trajectory = trajectory;
  r.waypoints = waypoints;
  return r;
}

std::map<std::string, double> jointStateMsgToMap(
    const sensor_msgs::msg::JointState &state) {
  std::map<std::string, double> out;
  for (std::size_t i = 0; i < state.name.size(); ++i) {
    out.emplace(state.name[i], state.position[i]);
  }
  return out;
}

std::vector<std::map<std::string, double>> jointStateArrayToArrayOfMaps(
    const std::vector<double> &trajectory,
    const std::vector<std::string> &names) {
  std::vector<std::map<std::string, double>> out;
  if (trajectory.size() % names.size() != 0) {
    RCLCPP_ERROR(LOGGER, "Can't convert trajectory!");
  }
  const size_t total_size = trajectory.size();
  const size_t joints_size = names.size();
  const size_t max_idx = total_size / joints_size;
  size_t trajectory_idx = 0;
  out.resize(max_idx);
  for (std::size_t i = 0; i < max_idx && trajectory_idx < total_size; ++i) {
    for (std::size_t j = 0; j < joints_size; ++j) {
      out[i][names[j]] = trajectory[trajectory_idx++];
    }
  }
  return out;
}

void ReachDatabase::save(const std::string &filename) const {
  std::lock_guard<std::mutex> lock{mutex_};
  reach_msgs::msg::ReachDatabase msg = toReachDatabase(map_, results_);

  if (!reach::utils::toFile(filename, msg)) {
    throw std::runtime_error("Unable to save database to file: " + filename);
  }
}

bool ReachDatabase::load(const std::string &filename) {
  reach_msgs::msg::ReachDatabase msg;
  if (!reach::utils::fromFile(filename, msg)) {
    RCLCPP_ERROR(LOGGER, "Unable to serialize from file '%s'!",
                 filename.c_str());
    return false;
  }
  std::lock_guard<std::mutex> lock{mutex_};

  for (const auto &r : msg.records) {
    putHelper(r);
    results_.reach_percentage = msg.reach_percentage;
    results_.total_pose_score = msg.total_pose_score;
    results_.norm_total_pose_score = msg.norm_total_pose_score;
    results_.avg_num_neighbors = msg.avg_num_neighbors;
    results_.avg_joint_distance = msg.avg_joint_distance;
  }
  return true;
}

std::optional<reach_msgs::msg::ReachRecord> ReachDatabase::get(
    const std::string &id) const {
  std::lock_guard<std::mutex> lock{mutex_};
  auto it = map_.find(id);
  if (it != map_.end()) {
    return {it->second};
  } else {
    return {};
  }
}

void ReachDatabase::put(const reach_msgs::msg::ReachRecord &record) {
  std::lock_guard<std::mutex> lock{mutex_};
  return putHelper(record);
}

void ReachDatabase::putHelper(const reach_msgs::msg::ReachRecord &record) {
  map_[record.id] = record;
}

std::size_t ReachDatabase::size() const { return map_.size(); }

void ReachDatabase::calculateResults() {
  unsigned int success = 0, total = 0;
  double score = 0.0;
  for (int i = 0; i < this->size(); ++i) {
    reach_msgs::msg::ReachRecord msg = *this->get(std::to_string(i));

    if (msg.reached) {
      success++;
      score += msg.score;
    }

    total++;
  }
  const float pct_success =
      static_cast<float>(success) / static_cast<float>(total);

  results_.reach_percentage = 100.0f * pct_success;
  results_.total_pose_score = score;
  results_.norm_total_pose_score = score / pct_success;
}

void ReachDatabase::printResults() {
  RCLCPP_INFO(LOGGER, "------------------------------------------------");
  RCLCPP_INFO_STREAM(LOGGER, "Percent Reached = " << results_.reach_percentage);
  RCLCPP_INFO_STREAM(LOGGER,
                     "Total points score = " << results_.total_pose_score);
  RCLCPP_INFO_STREAM(LOGGER, "Normalized total points score = "
                                 << results_.norm_total_pose_score);
  RCLCPP_INFO_STREAM(
      LOGGER, "Average reachable neighbors = " << results_.avg_num_neighbors);
  RCLCPP_INFO_STREAM(
      LOGGER, "Average joint distance = " << results_.avg_joint_distance);
  RCLCPP_INFO_STREAM(LOGGER,
                     "------------------------------------------------");
}

reach_msgs::msg::ReachDatabase ReachDatabase::toReachDatabaseMsg() {
  return toReachDatabase(map_, results_);
}

}  // namespace core
}  // namespace reach
