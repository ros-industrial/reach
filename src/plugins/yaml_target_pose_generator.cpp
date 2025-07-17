#include <reach/plugins/yaml_target_pose_generator.h>
#include <reach/plugin_utils.h>
#include <reach/types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointField.h>
#include <yaml-cpp/yaml.h>
using namespace reach;

static VectorIsometry3d parseYAML(YAML::Node pose_file)
{
  VectorIsometry3d target_poses;
  for (const auto& pose : pose_file["poses"])
  {
    auto position = pose["position"];
    auto orientation = pose["orientation"];

    Eigen::Isometry3d target_pose;
    target_pose.translation() =
        Eigen::Vector3d(position['x'].as<double>(), position['y'].as<double>(), position['z'].as<double>());
    target_pose.linear() = Eigen::Quaterniond(orientation['w'].as<double>(), orientation['x'].as<double>(),
                                              orientation['y'].as<double>(), orientation['z'].as<double>())
                               .toRotationMatrix();
    target_poses.push_back(target_pose);
  }
  return target_poses;
}

namespace reach
{
YAMLTargetPoseGenerator::YAMLTargetPoseGenerator(std::string filename) : filename_(resolveURI(filename))
{
}

VectorIsometry3d YAMLTargetPoseGenerator::generate() const
{
  // Check if file exists
  if (!boost::filesystem::exists(filename_))
    throw std::runtime_error("File '" + filename_ + "' does not exist");

  YAML::Node poses = YAML::LoadFile(filename_);
  VectorIsometry3d target_poses = parseYAML(poses);

  std::transform(target_poses.begin(), target_poses.end(), std::back_inserter(target_poses),
                 [](const Eigen::Isometry3d& pose) { return createFrame(pose.translation().cast<float>(), 
                  pose.rotation().cast<float>() * Eigen::Vector3f::UnitZ()); });
  return target_poses;
}

TargetPoseGenerator::ConstPtr YAMLTargetPoseGeneratorFactory::create(const YAML::Node& config) const
{
  return std::make_shared<YAMLTargetPoseGenerator>(get<std::string>(config, "poses"));
}

}  // namespace reach
