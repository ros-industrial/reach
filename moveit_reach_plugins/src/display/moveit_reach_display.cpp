#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <reach_plugins/display/moveit_reach_display.h>
#include <xmlrpcpp/XmlRpcException.h>

const static std::string PLANNING_SCENE_TOPIC = "planning_scene_display";

namespace reach_plugins
{
namespace display
{

MoveItReachDisplay::MoveItReachDisplay()
  : ReachDisplayBase()
{

}

bool MoveItReachDisplay::initialize(XmlRpc::XmlRpcValue& config)
{
  if(!config.hasMember("planning_group") ||
     !config.hasMember("collision_mesh_filename") ||
     !config.hasMember("collision_mesh_frame") ||
     !config.hasMember("fixed_frame") ||
     !config.hasMember("marker_scale"))
  {
    ROS_ERROR("MoveIt IK Solver Plugin is missing one or more configuration parameters");
    return false;
  }

  std::string planning_group;
  try
  {
    planning_group = std::string(config["planning_group"]);
    collision_mesh_filename_ = std::string(config["collision_mesh_filename"]);
    collision_mesh_frame_ = std::string(config["collision_mesh_frame"]);
    fixed_frame_ = std::string(config["fixed_frame"]);
    marker_scale_ = double(config["marker_scale"]);
  }
  catch(const XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR_STREAM(ex.getMessage());
    return false;
  }

  model_ = moveit::planning_interface::getSharedRobotModel("robot_description");
  if(!model_)
  {
    ROS_ERROR("Failed to initialize robot model pointer");
    return false;
  }

  jmg_ = model_->getJointModelGroup(planning_group);
  if(!jmg_)
  {
    ROS_ERROR_STREAM("Failed to get joint model group for '" << planning_group << "'");
    return false;
  }

  scene_.reset(new planning_scene::PlanningScene (model_));

  // Check that the input collision mesh frame exists
  if(!scene_->knowsFrameTransform(collision_mesh_frame_))
  {
    ROS_ERROR_STREAM("Specified collision mesh frame '" << collision_mesh_frame_ << "' does not exist");
    return false;
  }

  // Add the collision object to the planning scene
  const std::string object_name = "reach_object";
  moveit_msgs::CollisionObject obj = utils::createCollisionObject(collision_mesh_filename_, collision_mesh_frame_, object_name);
  if(!scene_->processCollisionObjectMsg(obj))
  {
    ROS_ERROR("Failed to add collision mesh to planning scene");
    return false;
  }

  scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>(PLANNING_SCENE_TOPIC, 1, true);

  ROS_INFO_STREAM("Successfully initialized MoveItReachDisplay plugin");
  return true;
}

void MoveItReachDisplay::showEnvironment()
{
  moveit_msgs::PlanningScene scene_msg;
  scene_->getPlanningSceneMsg(scene_msg);
  scene_pub_.publish(scene_msg);
}

void MoveItReachDisplay::updateRobotPose(const std::map<std::string, double>& pose)
{
  std::vector<std::string> joint_names = jmg_->getActiveJointModelNames();
  std::vector<double> joints;
  if(utils::transcribeInputMap(pose, joint_names, joints))
  {
    moveit_msgs::PlanningScene scene_msg;
    scene_msg.is_diff = true;
    scene_msg.robot_state.is_diff = true;
    scene_msg.robot_state.joint_state.name = joint_names;
    scene_msg.robot_state.joint_state.position = joints;
    scene_pub_.publish(scene_msg);
  }
  else
  {
    ROS_ERROR("Failed to transcribe input joints");
  }
}

} // namespace display
} // namespace reach_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(reach_plugins::display::MoveItReachDisplay, reach_plugins::display::ReachDisplayBase)
