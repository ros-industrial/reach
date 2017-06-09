#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointField.h>
#include <pcl_ros/point_cloud.h>
#include <robot_reach_study/SampleMesh.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

const static std::string SAMPLE_MESH_SRV_TOPIC = "sample_mesh";

bool getObjectTF(const std::string& world_frame,
                 const std::string& object_frame,
                 tf::StampedTransform& transform)
{
  tf::TransformListener listener;
  if(listener.waitForTransform(world_frame, object_frame, ros::Time::now(), ros::Duration(5.0)))
  {
    try
    {
      listener.lookupTransform(world_frame, object_frame, ros::Time(0.0), transform);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return false;
    }
  }
  else
  {
    ROS_ERROR("TF lookup between %s and %s has timed out", world_frame.c_str(), object_frame.c_str());
    return false;
  }
  return true;
}

bool hasNormals(pcl::PCLPointCloud2& cloud)
{
  auto nx = std::find_if(cloud.fields.begin(), cloud.fields.end(), [] (pcl::PCLPointField& field) {return field.name == "normal_x";});
  auto ny = std::find_if(cloud.fields.begin(), cloud.fields.end(), [] (pcl::PCLPointField& field) {return field.name == "normal_y";});
  auto nz = std::find_if(cloud.fields.begin(), cloud.fields.end(), [] (pcl::PCLPointField& field) {return field.name == "normal_z";});

  if(nx == cloud.fields.end() || ny == cloud.fields.end() || nz == cloud.fields.end())
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool getSampledMesh(robot_reach_study::SampleMesh::Request& req,
                    robot_reach_study::SampleMesh::Response& res)
{
  // Check if file exists
  if(!boost::filesystem::exists(req.cloud_filename))
  {
    ROS_ERROR("%s does not exist", req.cloud_filename.c_str());
    return false;
  }

  pcl::PCLPointCloud2 cloud_msg;
  if(pcl::io::loadPCDFile(req.cloud_filename, cloud_msg) == -1)
  {
    return false;
  }

  if(!hasNormals(cloud_msg))
  {
    return false;
  }

  pcl::PointCloud<pcl::PointNormal> cloud;
  pcl::fromPCLPointCloud2(cloud_msg, cloud);

  // Transform point cloud to correct frame
  tf::StampedTransform object_tf;
  if(!getObjectTF(req.world_frame, req.object_frame, object_tf))
  {
    return false;
  }

  Eigen::Affine3d transform;
  tf::transformTFToEigen(object_tf, transform);
  pcl::PointCloud<pcl::PointNormal> transformed_cloud;
  pcl::transformPointCloudWithNormals(cloud, transformed_cloud, transform.matrix());

  // Convert point cloud to message for output
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(transformed_cloud, res.cloud);

  return true;
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "sample_mesh_server");

  // Create a ROS node handle
  ros::NodeHandle nh;

  // Create a server
  ros::ServiceServer service = nh.advertiseService(SAMPLE_MESH_SRV_TOPIC, getSampledMesh);

  ros::spin();

  return 0;
}
