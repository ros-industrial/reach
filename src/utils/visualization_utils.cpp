#include "reach/utils/visualization_utils.h"
#include <eigen_conversions/eigen_msg.h>
#include <pcl/features/moment_of_inertia_estimation.h>

const static double ARROW_SCALE_RATIO = 6.0;
const static double NEIGHBOR_MARKER_SCALE_RATIO = ARROW_SCALE_RATIO / 2.0;

namespace reach
{
namespace utils
{

visualization_msgs::Marker makeVisual(const robot_reach_study::ReachRecord& r,
                                      const std::string& frame,
                                      const double scale,
                                      const std::string& ns = "reach",
                                      const boost::optional<std::vector<float>>& color = {})
{
  static int idx = 0;

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = idx++;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  Eigen::Affine3d goal_eigen;
  tf::poseMsgToEigen(r.goal, goal_eigen);

  // Transform arrow such that arrow x-axis points along goal pose z-axis (Rviz convention)
  // convert msg parameter goal to Eigen matrix
  Eigen::AngleAxisd rot_flip_normal (M_PI, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rot_x_to_z (-M_PI / 2, Eigen::Vector3d::UnitY());

  // Transform
  goal_eigen = goal_eigen * rot_flip_normal * rot_x_to_z;

  // Convert back to geometry_msgs pose
  geometry_msgs::Pose msg;
  tf::poseEigenToMsg(goal_eigen, msg);
  marker.pose = msg;

  marker.scale.x = scale;
  marker.scale.y = scale / ARROW_SCALE_RATIO;
  marker.scale.z = scale / ARROW_SCALE_RATIO;

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
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    }
    else
    {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }
  }

  return marker;
}

visualization_msgs::InteractiveMarker makeInteractiveMarker(const robot_reach_study::ReachRecord& r,
                                                            const std::string& frame,
                                                            const double scale)
{
  visualization_msgs::InteractiveMarker m;
  m.header.frame_id = frame;
  m.name = r.id;

  // Control
  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  // Visuals
  auto visual = makeVisual(r, frame, scale);
  control.markers.push_back(visual);
  m.controls.push_back(control);

  return m;
}

visualization_msgs::Marker makeMarker(const std::vector<geometry_msgs::Point>& pts,
                                      const std::string& frame,
                                      const double scale,
                                      const std::string& ns = "")
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = marker.scale.y = marker.scale.z = scale / NEIGHBOR_MARKER_SCALE_RATIO;

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

double getMajorLength(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
  pcl::MomentOfInertiaEstimation<pcl::PointNormal> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.compute();

  pcl::PointNormal min_pt, max_pt, position;
  Eigen::Matrix3f rotation;
  feature_extractor.getOBB(min_pt, max_pt, position, rotation);

  std::vector<double> lengths;
  lengths.push_back(max_pt.x - min_pt.x);
  lengths.push_back(max_pt.y - min_pt.y);
  lengths.push_back(max_pt.z - min_pt.z);

  return *(std::max_element(lengths.begin(), lengths.end()));
}

} // namespace utils
} // namespace reach

