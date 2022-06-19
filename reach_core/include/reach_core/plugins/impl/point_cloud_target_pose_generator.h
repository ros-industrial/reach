#include <reach_core/plugins/target_pose_generator_base.h>

namespace reach
{
namespace plugins
{
class PointCloudTargetPoseGenerator : public TargetPoseGeneratorBase
{
  VectorIsometry3d generate() const override;

  void initialize(const XmlRpc::XmlRpcValue& config) override;

private:
  std::string filename_;
};

} // namespace plugins
} // namespace reach

