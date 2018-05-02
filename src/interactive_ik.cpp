#include <robot_reach_study/interactive_ik.h>
#include <robot_reach_study/ik_helper.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <robot_reach_study/utils.h>

const static std::string ROBOT_STATE_TOPIC = "robot_state";
const static std::string REACH_NEIGHBORS_TOPIC = "reach_neighbors";
const static std::string REACH_COMPARISON_TOPIC = "reach_comparison";
const static std::string PLANNING_SCENE_TOPIC = "scene";
const static std::string INTERACTIVE_MARKER_TOPIC = "reach_int_markers";
const static float RE_SOLVE_IK_ATTEMPTS = 3;
const static float RE_SOLVE_IK_TIMEOUT = 0.2;

robot_reach_study::InteractiveIK::InteractiveIK(ros::NodeHandle& nh,
                                                std::shared_ptr<robot_reach_study::Database>& db,
                                                std::shared_ptr<robot_reach_study::IkHelper>& ik_helper)
  : server_{INTERACTIVE_MARKER_TOPIC, "", false}
{
  // Generate interactive marker
  menu_handler_.insert("Show Result", [this] (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb) {
    this->showResultCB(fb);
  });
  menu_handler_.insert("Show Seed Position", [this] (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb) {
    this->showSeedCB(fb);
  });
  menu_handler_.insert("Re-solve IK", [this] (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb) {
    this->reSolveIKCB(fb);
  });
  menu_handler_.insert("Show Reach to Neighbors (Direct)", [this] (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb) {
    this->reachNeighborsDirectCB(fb);
  });
  menu_handler_.insert("Show Reach to Neighbors (Recursive)", [this] (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb) {
    this->reachNeighborsRecursiveCB(fb);
  });

  db_ = db;
  ik_helper_ = ik_helper;

  nh_ = nh;
  state_pub_ = nh.advertise<moveit_msgs::DisplayRobotState>(ROBOT_STATE_TOPIC, 1, true);
  neighbor_pub_ = nh.advertise<visualization_msgs::Marker>(REACH_NEIGHBORS_TOPIC, 1, true);
  diff_pub_ = nh.advertise<visualization_msgs::MarkerArray>(REACH_COMPARISON_TOPIC, 1, true);
  scene_pub_ = nh.advertise<moveit_msgs::PlanningScene>(PLANNING_SCENE_TOPIC, 1, true);
}

void robot_reach_study::InteractiveIK::createReachMarkers()
{
  for (const auto& m : *db_)
  {
    addRecord(m.second);
  }
  server_.applyChanges();
}

void robot_reach_study::InteractiveIK::publishScene(const moveit_msgs::PlanningScene& msg)
{
  scene_pub_.publish(msg);
}

void robot_reach_study::InteractiveIK::reSolveIKCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &fb)
{
  auto lookup = db_->get(fb->marker_name);
  if (lookup)
  {
    // Set the robot's seed state to the current robot state
    moveit::core::RobotState goal_state (ik_helper_->getCurrentRobotState());
    moveit::core::RobotState seed_state (ik_helper_->getCurrentRobotState());
    moveit::core::robotStateMsgToRobotState(lookup->goal_state, goal_state);
    moveit::core::robotStateMsgToRobotState(lookup->seed_state, seed_state);

    // Save the original IK solution attempts and timeout
    const int sol_attempts = ik_helper_->getSolutionAttempts();
    const float sol_timeout = ik_helper_->getSolutionTimeout();

    // Increase the number of solution attempts and the timeout
    ik_helper_->setSolutionAttempts(RE_SOLVE_IK_ATTEMPTS);
    ik_helper_->setSolutionTimeout(RE_SOLVE_IK_TIMEOUT);

    // Re-solve IK at the selected marker
    boost::optional<double> score = ik_helper_->solveIKFromSeed(lookup->goal, seed_state, goal_state);

    // Update the database if the IK solution was valid
    if(score)
    {
      ROS_INFO("Solution found for point");

      lookup->reached = true;
      lookup->score = *score;
      moveit::core::robotStateToRobotStateMsg(goal_state, lookup->goal_state);

      // Update the interactive marker server
      server_.erase(fb->marker_name);
      addRecord(*lookup);
      server_.applyChanges();

      db_->put(*lookup);

      moveit_msgs::PlanningScene msg;
      msg.robot_state = lookup->goal_state;
      msg.is_diff = true;
      scene_pub_.publish(msg);
    }
    else
    {
      ROS_INFO("No solution found for point");
    }

    // Reset the original number of solution attempts and solution timeout
    ik_helper_->setSolutionAttempts(sol_attempts);
    ik_helper_->setSolutionTimeout(sol_timeout);
  }
}

void robot_reach_study::InteractiveIK::showResultCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &fb)
{
  auto lookup = db_->get(fb->marker_name);
  if (lookup)
  {
    auto& goal_state = lookup->goal_state;
    moveit_msgs::PlanningScene msg;
    msg.robot_state = goal_state;
    msg.is_diff = true;
    scene_pub_.publish(msg);
  }
}

void robot_reach_study::InteractiveIK::showSeedCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &fb)
{
  auto lookup = db_->get(fb->marker_name);
  if (lookup)
  {
    const auto& seed_state = lookup->seed_state;
    moveit_msgs::PlanningScene msg;
    msg.robot_state = seed_state;
    msg.is_diff = true;
    scene_pub_.publish(msg);
  }
}

void robot_reach_study::InteractiveIK::reachNeighborsDirectCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &fb)
{
  auto lookup = db_->get(fb->marker_name);
  if(!lookup)
  {
    ROS_ERROR("Object does not exist in database");
  }

  robot_reach_study::ReachRecord& reach_record = *lookup;
  std::vector<std::string> reached_pts = ik_helper_->reachNeighborsDirect(db_, reach_record);

  publishMarkerArray(reached_pts);
  ROS_INFO("%lu points are reachable from this pose", reached_pts.size());
  showResultCB(fb);
}

void robot_reach_study::InteractiveIK::reachNeighborsRecursiveCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &fb)
{
  auto lookup = db_->get(fb->marker_name);
  if(!lookup)
  {
    ROS_ERROR("Object does not exist in database");
  }

  robot_reach_study::ReachRecord& rec = *lookup;
  std::vector<std::string> reached_pts;
  double joint_distance = 0.0;

  const std::string kin_jmg_name = ik_helper_->getKinematicJointModelGroupName();
  const std::string manip_jmg_name = ik_helper_->getManipulabilityJointModelGroupName();
  ik_helper_->setKinematicJointModelGroup(manip_jmg_name);
  ik_helper_->reachNeighborsRecursive(db_, rec, reached_pts, joint_distance);
  ik_helper_->setKinematicJointModelGroup(kin_jmg_name);

  publishMarkerArray(reached_pts);
  ROS_INFO("%lu points are reachable from this pose", reached_pts.size());
  ROS_INFO("Total joint distance to all neighbors: %f", joint_distance);
  showResultCB(fb);
}

void robot_reach_study::InteractiveIK::reachDiffVisualizer(std::vector<std::pair<std::string, std::shared_ptr<robot_reach_study::Database>>> data)
{

  const int db_num = static_cast<int>(data.size());
  const int n_perm = pow(2, db_num);

  // Check that all databases are the same size
  for(int i = 0; i < db_num - 1; ++i)
  {
    if(data[i].second->count() != data[i + 1].second->count())
    {
      ROS_FATAL("Mismatched database sizes");
    }
  }

  // Generate a list of all possible namespace permutations
  std::vector<std::string> ns_vec(n_perm);
  ns_vec[0] = "not_all";
  ns_vec[n_perm - 1] = "all";

  for(char perm_ind = 1; perm_ind < static_cast<char>(n_perm - 1); ++perm_ind)
  {
    std::string ns_name = "";
    for(int i = 0; i < db_num; ++i)
    {
      if(((perm_ind >> i) & 1) == 1)
      {
        if(ns_name == "")
        {
          ns_name += data[i].first;
        }
        else
        {
          ns_name += "_AND_" + data[i].first;
        }
      }
    }
    ns_vec[static_cast<int>(perm_ind)] = ns_name;
  }

  // Set the color of the arrow display
  std::vector<float> color(4);
  color[0] = 1.0;
  color[1] = 0.0;
  color[2] = 0.0;
  color[3] = 1.0;
  boost::optional<std::vector<float>> arrow_color = color;

  // Create Rviz marker array
  visualization_msgs::MarkerArray marker_array;

  // Iterate over all points in the databases
  for(int i = 0; i < data[0].second->count(); ++i)
  {
    // Get the reach record message for the current point from each database
    std::vector<robot_reach_study::ReachRecord> msgs;
    for(int j = 0; j < db_num; ++j)
    {
      std::string msg_id = std::to_string(i);
      boost::optional<robot_reach_study::ReachRecord> msg = data[j].second->get(msg_id);
      msgs.push_back(*msg);
    }

    // Create a binary code based on whether the point was reached
    // code LSB is msg.reach boolean of 1st database
    // code << n is is msg.reach boolean of (n+1)th database
    char code = 0;
    for(int j = 0; j < db_num; ++j)
    {
      code += static_cast<char>(msgs[j].reached) << j;
    }

    // Create Rviz marker unless the point was reached by all or none of the robot configurations
    if(code != 0 && code != n_perm - 1)
    {
      std::string ns = {ns_vec[static_cast<int>(code)]};
      visualization_msgs::Marker arrow_marker = utils::makeVisual(msgs[0], fixed_frame_, marker_scale_, ns, {arrow_color});
      marker_array.markers.push_back(arrow_marker);
    }
  }
  diff_pub_.publish(marker_array);
}

void robot_reach_study::InteractiveIK::publishMarkerArray(std::vector<std::string>& msg_ids)
{
  std::vector<geometry_msgs::Point> pt_array;
  if(msg_ids.size() > 0)
  {
    for (auto it = msg_ids.begin(); it != msg_ids.end(); ++it)
    {
      // Update the interactive markers that have changed
      robot_reach_study::ReachRecord msg = *db_->get(*it);
      addRecord(msg);

      // Visualize points reached around input point
      geometry_msgs::Point pt;
      pt.x = msg.goal.position.x;
      pt.y = msg.goal.position.y;
      pt.z = msg.goal.position.z;
      pt_array.push_back(pt);
    }
  }

  // Create points marker, publish it, and move robot to result state for  given point
  server_.applyChanges();
  visualization_msgs::Marker pt_marker = utils::makeMarker(pt_array, fixed_frame_, marker_scale_);
  neighbor_pub_.publish(pt_marker);
}

void robot_reach_study::InteractiveIK::addRecord(const robot_reach_study::ReachRecord &rec)
{
  auto id = rec.id;
  auto marker = utils::makeInteractiveMarker(rec, fixed_frame_, marker_scale_);
  server_.insert(marker);
  menu_handler_.apply(server_, id);
}
