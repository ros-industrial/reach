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

#include <filesystem>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointField.h>
//#include <pcl_ros/point_cloud.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <reach_msgs/srv/load_point_cloud.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

constexpr char SAMPLE_MESH_SRV_TOPIC[] = "sample_mesh";

using LoadPCLSrv = reach_msgs::srv::LoadPointCloud;
using LoadPCLReq  = reach_msgs::srv::LoadPointCloud_Request;
using LoadPCLReqSharedPtr = LoadPCLReq::SharedPtr;
using LoadPCLRes  = reach_msgs::srv::LoadPointCloud_Response;
using LoadPCLResSharedPtr = LoadPCLRes::SharedPtr;


    class PointCloudServerNode : public rclcpp::Node {
    public:
        explicit PointCloudServerNode(const std::string &node_name) : Node(node_name) {

            server_ = this->create_service<LoadPCLSrv>(SAMPLE_MESH_SRV_TOPIC, [this](const LoadPCLReqSharedPtr req,
                                                                                     LoadPCLResSharedPtr res){

                RCLCPP_INFO(this->get_logger(), "Service callback started!");


                // getSampledMesh callback
                // Check if file exists
                if (!std::filesystem::exists(req->cloud_filename)) {
                    res->message = "File '" + req->cloud_filename + "' does not exist";
                    res->success = false;
                    return false;
                }

                pcl::PCLPointCloud2 cloud_msg;
                if (pcl::io::loadPCDFile(req->cloud_filename, cloud_msg) == -1) {
                    res->message = "Unable to load point cloud from '" + req->cloud_filename + "'";
                    res->success = false;
                    return false;
                }

                if (!hasNormals(cloud_msg)) {
                    res->message = "Point cloud file does not contain normals. Please regenerate the cloud with "
                                   "normal vectors";
                    res->success = false;
                    return true;
                }

                pcl::PointCloud<pcl::PointNormal> cloud;
                pcl::fromPCLPointCloud2(cloud_msg, cloud);

                // Transform point cloud to correct frame
                tf2_ros::Buffer buffer(this->get_clock());
                tf2_ros::TransformListener listener(buffer);
                Eigen::Isometry3d transform;
                try {
                    RCLCPP_INFO(this->get_logger(), "Try to look for transform!");
                    geometry_msgs::msg::TransformStamped tf = buffer.lookupTransform(req->fixed_frame,
                                                                                     req->object_frame,
                                                                                     rclcpp::Time(0),
                                                                                     rclcpp::Duration::from_seconds(5.0));
                    transform = tf2::transformToEigen(tf.transform);
                    RCLCPP_INFO(this->get_logger(), "x = %f  y = %f  z = %f ", tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
                    RCLCPP_INFO(this->get_logger(), "qx = %f  qy = %f  qz = %f  qw = %f ", tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
                                tf.transform.rotation.w);

                }
                catch (const tf2::TransformException &ex) {
                    RCLCPP_ERROR(this->get_logger(), "Catch tf exception!");

                    res->message = ex.what();
                    res->success = false;
                    RCLCPP_ERROR(this->get_logger(), "'%s'", ex.what());
                    return false;
                }
                catch(const rclcpp::exceptions::RCLError &exerr){
                    RCLCPP_ERROR(this->get_logger(), "Catch RCLError exception!");

                    RCLCPP_ERROR(this->get_logger(), "'%s'", exerr.what());
                    res->success = false;
                    res->message = exerr.what();
                    return false;
                }

                pcl::PointCloud<pcl::PointNormal> transformed_cloud;
                pcl::transformPointCloudWithNormals(cloud, transformed_cloud, transform.matrix());

                // Convert point cloud to message for output
                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(transformed_cloud, res->cloud);
                for(size_t i = 0; i < res->cloud.data.size(); ++i){
                    if (res->cloud.data[i]!= 0.0) {
                        RCLCPP_INFO(this->get_logger(), "%d-th data = %f ", i, res->cloud.data[i]);
                    }
                }

                res->success = true;
                res->message = "Successfully loaded point cloud from '" + req->cloud_filename + "'";

                RCLCPP_INFO(this->get_logger(), "Service callback finished!");

                return true;
            });
            server_->get_service_name();
        }

    private:

        rclcpp::Service<LoadPCLSrv>::SharedPtr server_;

        bool hasNormals(pcl::PCLPointCloud2 &cloud) {
            auto nx = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                                   [](pcl::PCLPointField &field) { return field.name == "normal_x"; });
            auto ny = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                                   [](pcl::PCLPointField &field) { return field.name == "normal_y"; });
            auto nz = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                                   [](pcl::PCLPointField &field) { return field.name == "normal_z"; });

            if (nx == cloud.fields.end() || ny == cloud.fields.end() || nz == cloud.fields.end()) {
                return false;
            } else {
                return true;
            }
        }


    };

int main(int argc, char **argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  // create node
  auto node = std::make_shared<PointCloudServerNode>("sample_mesh_server");
  // spin
  rclcpp::spin(node);

  return 0;
}
