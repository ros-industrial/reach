#ifndef UTILS_H
#define UTILS_H

#include <atomic>
#include <fstream>
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <moveit_msgs/CollisionObject.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <robot_reach_study/ReachRecord.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <visualization_msgs/InteractiveMarker.h>

const static double ARROW_SCALE_RATIO = 6.0;
const static double NEIGHBOR_MARKER_SCALE_RATIO = ARROW_SCALE_RATIO / 2.0;

namespace robot_reach_study
{
namespace utils
{
  // Write a message to disk
  template <class T> inline bool toFile(const std::string& path, const T& msg)
  {
    namespace ser = ros::serialization;
    uint32_t serialize_size = ser::serializationLength(msg);
    boost::shared_array<uint8_t> buffer(new uint8_t[serialize_size]);

    ser::OStream stream(buffer.get(), serialize_size);
    ser::serialize(stream, msg);

    std::ofstream file(path.c_str(), std::ios::out | std::ios::binary);
    if (!file)
    {
      return false;
    }
    else
    {
      file.write((char*)buffer.get(), serialize_size);
      return file.good();
    }
  }

  // Restore a message from disk
  template <class T> inline bool fromFile(const std::string& path, T& msg)
  {
    namespace ser = ros::serialization;

    std::ifstream ifs(path.c_str(), std::ios::in | std::ios::binary);
    if (!ifs)
    {
      return false;
    }

    ifs.seekg(0, std::ios::end);
    std::streampos end = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    std::streampos begin = ifs.tellg();

    uint32_t file_size = end - begin;

    boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
    ifs.read((char*)ibuffer.get(), file_size);
    ser::IStream istream(ibuffer.get(), file_size);
    ser::deserialize(istream, msg);
    return true;
  }

  // Create a pose with the Z-axis as the normal and an arbitrary orientation
  inline Eigen::Affine3d createFrame(const Eigen::Vector3f& pt,
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

  // Print the progress of a multi-threaded process in integers
  inline void integerProgressPrinter(std::atomic<int>& current_counter, std::atomic<int>& previous_pct, const int total_size)
  {
    const float current_pct_float = (static_cast<float>(current_counter.load()) / static_cast<float>(total_size)) * 100.0;
    const int current_pct = static_cast<int>(current_pct_float);
    if(current_pct > previous_pct.load())
    {
      ROS_INFO("[%d%%]", current_pct);
    }
    previous_pct = current_pct;
  }

  // Create an arrow marker for visualizing the reach study results in Rviz
  inline visualization_msgs::Marker makeVisual(const robot_reach_study::ReachRecord& r,
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

  inline visualization_msgs::InteractiveMarker makeInteractiveMarker(const robot_reach_study::ReachRecord& r,
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

  inline visualization_msgs::Marker makeMarker(const std::vector<geometry_msgs::Point>& pts,
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

  inline moveit_msgs::CollisionObject createCollisionObject(const std::string& mesh_filename,
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

  inline float getMajorLength(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
  {
    pcl::MomentOfInertiaEstimation<pcl::PointNormal> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    float major_len, middle_len, minor_len;
    feature_extractor.getEigenValues(major_len, middle_len, minor_len);

    return 2.0 * major_len;
  }

}
}

#endif // UTILS_H
