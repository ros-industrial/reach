#include <reach_plugins/utils.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <ros/console.h>

namespace reach_plugins
{
namespace utils
{

moveit_msgs::CollisionObject createCollisionObject(const std::string& mesh_filename,
                                                   const std::string& parent_link,
                                                   const std::string& object_name)
{
  // Create a CollisionObject message for the reach object
  moveit_msgs::CollisionObject obj;
  obj.header.frame_id = parent_link;
  obj.id = object_name;
  shapes::ShapeMsg shape_msg;
  shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_filename);
  shapes::constructMsgFromShape(mesh, shape_msg);
  obj.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));
  obj.operation = obj.ADD;

  // Assign a default pose to the mesh
  geometry_msgs::Pose pose;
  pose.position.x = pose.position.y = pose.position.z = 0.0;
  pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  obj.mesh_poses.push_back(pose);

  return obj;
}

} // namespace utils
} // namespace reach_plugins
