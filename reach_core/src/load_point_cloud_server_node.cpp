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
#include <reach_msgs/LoadPointCloud.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

const static std::string SAMPLE_MESH_SRV_TOPIC = "sample_mesh";

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

bool getSampledMesh(reach_msgs::LoadPointCloudRequest& req,
                    reach_msgs::LoadPointCloudResponse& res)
{
  // Check if file exists
  if(!boost::filesystem::exists(req.cloud_filename))
  {
    res.message = "File '" + req.cloud_filename + "' does not exist";
    res.success = false;

    return true;
  }

  pcl::PCLPointCloud2 cloud_msg;
  if(pcl::io::loadPCDFile(req.cloud_filename, cloud_msg) == -1)
  {
    res.message = "Unable to load point cloud from '" + req.cloud_filename + "'";
    res.success = false;
    return true;
  }

  if(!hasNormals(cloud_msg))
  {
    res.message = "Point cloud file does not contain normals. Please regenerate the cloud with "
                  "normal vectors";
    res.success = false;
    return true;
  }

  pcl::PointCloud<pcl::PointNormal> cloud;
  pcl::fromPCLPointCloud2(cloud_msg, cloud);

  // Transform point cloud to correct frame
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  Eigen::Isometry3d transform;
  try
  {
    geometry_msgs::TransformStamped tf = buffer.lookupTransform(req.fixed_frame,
                                                                req.object_frame,
                                                                ros::Time(0),
                                                                ros::Duration(5.0));
    transform = tf2::transformToEigen(tf.transform);
  }
  catch(const tf2::TransformException& ex)
  {
    res.message = ex.what();
    res.success = false;
    return true;
  }

  pcl::PointCloud<pcl::PointNormal> transformed_cloud;
  pcl::transformPointCloudWithNormals(cloud, transformed_cloud, transform.matrix());

  // Convert point cloud to message for output
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(transformed_cloud, res.cloud);

  res.success = true;
  res.message = "Successfully loaded point cloud from '" + req.cloud_filename + "'";

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
