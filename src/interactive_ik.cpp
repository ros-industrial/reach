#include <robot_reach_study/interactive_ik.h>
#include <robot_reach_study/ik_helper.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/tools.h>
#include <ros/console.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>


namespace
{

static visualization_msgs::Marker makeVisual(const robot_reach_study::ReachRecord& r,
                                             const std::string ns = "reach",
                                             const boost::optional<std::vector<float>>& color = {})
{
  static int idx = 0;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = idx++;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  // Transform arrow such that arrow x-axis points along goal pose z-axis (Rviz convention)
  // convert msg parameter goal to Eigen matrix
  Eigen::Affine3d goal_eigen;
  tf::poseMsgToEigen(r.goal, goal_eigen);

  Eigen::AngleAxisd rot_flip_normal (M_PI, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rot_x_to_z (-M_PI / 2, Eigen::Vector3d::UnitY());

  // Transform
  goal_eigen = goal_eigen * rot_flip_normal * rot_x_to_z;

  // Convert back to geometry_msgs pose
  geometry_msgs::Pose msg;
  tf::poseEigenToMsg(goal_eigen, msg);
  marker.pose = msg;

  marker.scale.x = 0.150;
  marker.scale.y = 0.025;
  marker.scale.z = 0.025;

  if(color)
  {
    std::vector<float> color_vec = *color;
    marker.color.r = color_vec[0];
    marker.color.g = color_vec[1];
    marker.color.b = color_vec[2];
    marker.color.a = color_vec[3];
  }
  else
  {
    marker.color.a = 1.0; // Don't forget to set the alpha!

    if (r.reached)
    {
      marker.color.r = 0;
      marker.color.g = 0;
      marker.color.b = 1.0;
    }
    else
    {
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;
    }
  }

  return marker;
}

visualization_msgs::InteractiveMarker makeInteractiveMarker(const robot_reach_study::ReachRecord& r)
{
  visualization_msgs::InteractiveMarker m;
  m.header.frame_id = "world";
  m.scale = 1.0;
  m.name = r.id;

  // Control
  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  // Visuals
  auto visual = makeVisual(r);
  control.markers.push_back(visual);
  m.controls.push_back(control);

  return m;
}

visualization_msgs::Marker makeMarker(std::vector<geometry_msgs::Point> pts, int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "line";
  marker.id = id;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0;
  marker.color.g = 1.0;
  marker.color.b = 0;

  for(std::size_t i = 0; i < pts.size(); ++i)
  {
    marker.points.push_back(pts[i]);
  }

  return marker;
}

}

robot_reach_study::InteractiveIK::InteractiveIK(std::shared_ptr<robot_reach_study::Database>& db,
                                                std::shared_ptr<robot_reach_study::IkHelper>& ik_helper)
  : server_{"reach", "", false}
//  , db_{db}
//  , ik_helper_{robot_reach_study::IkHelper("robot_description", "robot_rail", "robot_rail")}
{
  // Generate interactive marker
  menu_handler.insert("Show Result", [this] (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb) {
    this->showResultCB(fb);
  });
  menu_handler.insert("Show Seed Position", [this] (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb) {
    this->showSeedCB(fb);
  });
  menu_handler.insert("Re-solve IK", [this] (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb) {
    this->reSolveIKCB(fb);
  });
  menu_handler.insert("Show Reach to Neighbors (Direct)", [this] (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb) {
    this->reachNeighborsDirectCB(fb);
  });
  menu_handler.insert("Show Reach to Neighbors (Recursive)", [this] (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb) {
    this->reachNeighborsRecursiveCB(fb);
  });

  db_ = db;
  ik_helper_ = ik_helper;

  ros::NodeHandle nh;
  state_pub_ = nh.advertise<moveit_msgs::DisplayRobotState>("state", 0);
  line_pub_ = nh.advertise<visualization_msgs::Marker>("pt_marker", 1, true);
  diff_pub_ = nh.advertise<visualization_msgs::MarkerArray>("reach_diff", 1, true);
  scene_pub_ = nh.advertise<moveit_msgs::PlanningScene>("test", 1, true);

//  moveit_msgs::PlanningScene msg;
//  ik_helper_->getPlanningScene()->getPlanningSceneMsg(msg);
//  scene_pub_.publish(msg);

//  // Create markers
//  for (const auto& m : db)
//  {
//    addRecord(m.second);
//  }

//  server_.applyChanges();
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
    moveit::core::RobotState goal_state (ik_helper_->getCurrentRobotState()), seed_state (ik_helper_->getCurrentRobotState());

    moveit::core::robotStateMsgToRobotState(lookup->goal_state, goal_state);
    moveit::core::robotStateMsgToRobotState(lookup->seed_state, seed_state);

    ik_helper_->setSolutionAttempts(3);
    ik_helper_->setSolutionTimeout(0.2);
    boost::optional<double> score = ik_helper_->solveIKFromSeed(lookup->goal, seed_state, goal_state);
    if(score)
    {
      ROS_INFO("Solution found for point");

      // Update the database
      lookup->reached = true;
      lookup->score = *score;
      moveit::core::robotStateToRobotStateMsg(goal_state, lookup->goal_state);

      // Update the interactive marker server
      server_.erase(fb->marker_name);
      addRecord(*lookup);
      server_.applyChanges();

      db_->put(*lookup);

    }
    else
    {
      ROS_INFO("No solution found for point");
    }

    ik_helper_->setSolutionAttempts(1);
    ik_helper_->setSolutionTimeout(0.02);
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
  std::vector<std::string> msg_ids = ik_helper_->reachNeighborsDirect(db_, reach_record);
  publishMarkerArray(msg_ids);
  showResultCB(fb);
}

void robot_reach_study::InteractiveIK::reachNeighborsRecursiveCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &fb)
{
  auto lookup = db_->get(fb->marker_name);
  if(!lookup)
  {
    ROS_ERROR("Object does not exist in database");
  }

  robot_reach_study::ReachRecord& reach_record = *lookup;
  std::vector<std::string> msg_ids;
  moveit::core::RobotState goal_state(ik_helper_->getCurrentRobotState());
  moveit::core::robotStateMsgToRobotState(reach_record.goal_state, goal_state);

  ik_helper_->reachNeighborsRecursive(db_, reach_record.id, reach_record.goal, goal_state, msg_ids);
  publishMarkerArray(msg_ids);
  showResultCB(fb);
}

void robot_reach_study::InteractiveIK::reachDiffVisualizer(std::vector<std::pair<std::string, robot_reach_study::Database*>> data)
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
        ns_name += "_" + data[i].first;
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
      visualization_msgs::Marker arrow_marker = makeVisual(msgs[0], ns, {arrow_color});
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
  visualization_msgs::Marker pt_marker = makeMarker(pt_array, 0);
  line_pub_.publish(pt_marker);

  ROS_INFO("%lu points are reachable from this pose", pt_array.size());
}

void robot_reach_study::InteractiveIK::addRecord(const robot_reach_study::ReachRecord &rec)
{
  auto id = rec.id;
  auto marker = makeInteractiveMarker(rec);
  server_.insert(marker);
  menu_handler.apply(server_, id);
}
