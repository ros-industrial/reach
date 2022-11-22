#include <reach_core/interfaces/target_pose_generator.h>
#include <reach_core/plugin_utils.h>

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

namespace reach
{
class PointCloudTargetPoseGenerator : public TargetPoseGenerator
{
public:
  PointCloudTargetPoseGenerator(std::string filename) : filename_(resolveURI(filename))
  {
  }

  VectorIsometry3d generate() const override
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

private:
  std::string filename_;
};

struct PointCloudTargetPoseGeneratorFactory : public TargetPoseGeneratorFactory
{
  using TargetPoseGeneratorFactory::TargetPoseGeneratorFactory;
  TargetPoseGenerator::ConstPtr create(const YAML::Node& config) const override
  {
    return std::make_shared<PointCloudTargetPoseGenerator>(get<std::string>(config, "pcd_file"));
  }
};

}  // namespace reach

EXPORT_TARGET_POSE_GENERATOR_PLUGIN(reach::PointCloudTargetPoseGeneratorFactory, PointCloudTargetPoseGenerator)
