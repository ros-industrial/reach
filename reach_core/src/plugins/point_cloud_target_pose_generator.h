#include <reach_core/interfaces/target_pose_generator.h>

namespace reach
{
class PointCloudTargetPoseGenerator : public TargetPoseGenerator
{
  VectorIsometry3d generate() const override;

  void initialize(const YAML::Node& config) override;

private:
  std::string filename_;
};

} // namespace reach

