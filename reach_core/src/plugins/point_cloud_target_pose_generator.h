#include <reach_core/interfaces/target_pose_generator.h>

namespace reach
{
class PointCloudTargetPoseGenerator : public TargetPoseGenerator
{
public:
  PointCloudTargetPoseGenerator(std::string filename);
  VectorIsometry3d generate() const override;

private:
  std::string filename_;
};

struct PointCloudTargetPoseGeneratorFactory : public TargetPoseGeneratorFactory
{
  using TargetPoseGeneratorFactory::TargetPoseGeneratorFactory;
  TargetPoseGenerator::ConstPtr create(const YAML::Node& config) const override;
};

} // namespace reach

