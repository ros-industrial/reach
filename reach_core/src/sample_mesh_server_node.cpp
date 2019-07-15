/* 
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointField.h>
#include <pcl_ros/point_cloud.h>
#include <reach_msgs/SampleMesh.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

const static std::string SAMPLE_MESH_SRV_TOPIC = "sample_mesh";

bool getObjectTF(const std::string& fixed_frame,
                 const std::string& object_frame,
                 tf::StampedTransform& transform)
{
  tf::TransformListener listener;
  if(listener.waitForTransform(fixed_frame, object_frame, ros::Time::now(), ros::Duration(5.0)))
  {
    try
    {
      listener.lookupTransform(fixed_frame, object_frame, ros::Time(0.0), transform);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return false;
    }
  }
  else
  {
    ROS_ERROR("TF lookup between %s and %s has timed out", fixed_frame.c_str(), object_frame.c_str());
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

bool getSampledMesh(reach_msgs::SampleMeshRequest& req,
                    reach_msgs::SampleMeshResponse& res)
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
    ROS_ERROR("Unable to load point cloud .pcd file");
    return false;
  }

  if(!hasNormals(cloud_msg))
  {
    ROS_ERROR("Point cloud file does not contain normals. Please regenerate the cloud with normal vectors");
    return false;
  }

  pcl::PointCloud<pcl::PointNormal> cloud;
  pcl::fromPCLPointCloud2(cloud_msg, cloud);

  // Transform point cloud to correct frame
  tf::StampedTransform object_tf;
  if(!getObjectTF(req.fixed_frame, req.object_frame, object_tf))
  {
    return false;
  }

  Eigen::Isometry3d transform;
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
