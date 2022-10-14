#include "point_cloud_target_pose_generator.h"
#include <reach_core/utils.h>

#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointField.h>
#include <yaml-cpp/yaml.h>

static bool hasNormals(pcl::PCLPointCloud2& cloud)
{
  auto nx = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                         [](pcl::PCLPointField& field) { return field.name == "normal_x"; });
  auto ny = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                         [](pcl::PCLPointField& field) { return field.name == "normal_y"; });
  auto nz = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                         [](pcl::PCLPointField& field) { return field.name == "normal_z"; });

  if (nx == cloud.fields.end() || ny == cloud.fields.end() || nz == cloud.fields.end())
  {
    return false;
  }
  else
  {
    return true;
  }
}

static Eigen::Isometry3d createFrame(const Eigen::Vector3f& pt, const Eigen::Vector3f& norm)
{
  // Initialize coordinate frame and set XYZ location
  Eigen::Isometry3f p = Eigen::Isometry3f::Identity();
  p.matrix()(0, 3) = pt(0);
  p.matrix()(1, 3) = pt(1);
  p.matrix()(2, 3) = pt(2);

  // Create plane from point normal
  Eigen::Hyperplane<float, 3> plane(norm, Eigen::Vector3f(0, 0, 0));

  // If the normal and global x-axis are not closely aligned
  if (std::abs(norm.dot(Eigen::Vector3f::UnitX())) < 0.90)
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

namespace reach
{
PointCloudTargetPoseGenerator::PointCloudTargetPoseGenerator(std::string filename) : filename_(std::move(filename))
{
}

VectorIsometry3d PointCloudTargetPoseGenerator::generate() const
{
  // Check if file exists
  if (!boost::filesystem::exists(filename_))
    throw std::runtime_error("File '" + filename_ + "' does not exist");

  pcl::PCLPointCloud2 cloud_msg;
  if (pcl::io::loadPCDFile(filename_, cloud_msg) < 0)
    throw std::runtime_error("Failed to load point cloud from '" + filename_ + "'");

  if (!hasNormals(cloud_msg))
    throw std::runtime_error("Point cloud file does not contain normals. Please regenerate the cloud with normal "
                             "vectors");

  pcl::PointCloud<pcl::PointNormal> cloud;
  pcl::fromPCLPointCloud2(cloud_msg, cloud);

  VectorIsometry3d target_poses;
  target_poses.reserve(cloud.size());
  std::transform(cloud.begin(), cloud.end(), std::back_inserter(target_poses), [](const pcl::PointNormal& pt) {
    return createFrame(pt.getArray3fMap(), pt.getNormalVector3fMap());
  });

  return target_poses;
}

TargetPoseGenerator::ConstPtr PointCloudTargetPoseGeneratorFactory::create(const YAML::Node& config) const
{
  return boost::make_shared<PointCloudTargetPoseGenerator>(get<std::string>(config, "pcd_file"));
}

} // namespace reach

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(reach::PointCloudTargetPoseGeneratorFactory, reach::TargetPoseGeneratorFactory)
