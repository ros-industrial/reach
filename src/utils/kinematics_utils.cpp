#include "reach/utils/kinematics_utils.h"
#include <robot_reach_study/ReachRecord.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <ros/console.h>

namespace reach
{
namespace utils
{

Eigen::Affine3d createFrame(const Eigen::Vector3f& pt,
                            const Eigen::Vector3f& norm)
{
  // Initialize coordinate frame and set XYZ location
  Eigen::Affine3f p = Eigen::Affine3f::Identity();
  p.matrix()(0, 3) = pt(0);
  p.matrix()(1, 3) = pt(1);
  p.matrix()(2, 3) = pt(2);

  // Create plane from point normal
  Eigen::Hyperplane<float, 3> plane (norm, Eigen::Vector3f(0, 0, 0));

  // If the normal and global x-axis are not closely aligned
  if (std::abs(norm.dot(Eigen::Vector3f::UnitX()) < 0.90))
  {
    // Project the global x-axis onto the plane to generate the x-axis
    Eigen::Vector3f x_axis = plane.projection(Eigen::Vector3f::UnitX()).normalized();
    p.matrix().col(0).head<3>() = x_axis;
    p.matrix().col(1).head<3>() = norm.cross(x_axis);
    p.matrix().col(2).head<3>() = norm;
  }
  else
  {
    // Project the global y-axis onto the plane to generate the y-axis
    Eigen::Vector3f y_axis = plane.projection(Eigen::Vector3f::UnitY()).normalized();
    p.matrix().col(0).head<3>() = y_axis.cross(norm);
    p.matrix().col(1).head<3>() = y_axis;
    p.matrix().col(2).head<3>() = norm;
  }

  return p.cast<double>();
}

moveit_msgs::CollisionObject createCollisionObject(const std::string& mesh_filename,
                                                   const std::string& parent_link,
                                                   const std::vector<std::string>& touch_links,
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

double getManipulability(const moveit::core::RobotState& state,
                         const moveit::core::JointModelGroup* jmg)
{
  // Calculate manipulability of kinematic chain of input robot state
  // Create new robot state to avoid dirty link transforms
  moveit::core::RobotState temp_state(state);

  // Get the Jacobian matrix
  Eigen::MatrixXd jacobian = temp_state.getJacobian(jmg);

  // Calculate manipulability by multiplying Jacobian matrix singular values together
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  Eigen::MatrixXd singular_values = svd.singularValues();
  double m = 1.0;
  for(unsigned int i = 0; i < singular_values.rows(); ++i)
  {
    m *= singular_values(i, 0);
  }
  return m;
}

double getJointPenalty(const moveit::core::RobotState& state,
                       const moveit::core::JointModelGroup* jmg,
                       std::vector<std::vector<double>>& joint_limits)
{
  std::vector<double> max, min, current;
  min = joint_limits[0];
  max = joint_limits[1];

  // Get current joint values in jmg chain
  moveit::core::RobotState temp_state(state);
  temp_state.copyJointGroupPositions(jmg, current);

  double penalty = 1.0;
  for(std::size_t i = 0; i < max.size(); ++i)
  {
    double range = max[i] - min[i];
    penalty *= ((current[i] - min[i])*(max[i] - current[i])) / pow(range, 2);
  }
  return std::max(0.0, 1.0 - exp(-1.0 * penalty));
}

} // namespace utils
} // namespace reach
