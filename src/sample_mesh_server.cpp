#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_ros/point_cloud.h>
#include <robot_reach_study/SampleMesh.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <urdf/model.h>

const static std::string SAMPLE_MESH_SRV_TOPIC = "sample_mesh";
const static std::string MESH_FILENAME_PREFIX = "package://";

vtkSmartPointer<vtkPolyData> readSTLFile(std::string file)
{
  vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
  reader->SetFileName(file.c_str());
  reader->SetMerging(1);
  reader->Update();

  return reader->GetOutput();
}

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

bool editMeshFilename(std::string& filename)
{
  // Find the prefix to remove in the current filename
  size_t pos = filename.find(MESH_FILENAME_PREFIX);
  if(pos != std::string::npos)
  {
    // Erase prefix from filename
    filename.erase(pos, MESH_FILENAME_PREFIX.length());

    // Find the first "/" in the filename (indicating end of package name)
    size_t start_pos = filename.find_first_of("/", 0);

    // Create package name string and get the full package name file path
    std::string pkg_name = filename.substr(pos, start_pos);
    std::string pkg_path = ros::package::getPath(pkg_name);
    if(pkg_path.length() == 0)
    {
      // Save the iterator to the filename if the ROS package where it came from is not found
      ROS_ERROR("Package not found: %s", pkg_name.c_str());
      return false;
    }

    // Erase the package name from the front of the filename and replace with full package path
    filename.erase(0, start_pos);
    filename.insert(filename.begin(), pkg_path.begin(), pkg_path.end());
  }
  else
  {
    ROS_ERROR("Mesh filename not in correct format: %s", filename.c_str());
    return false;
  }
  return true;
}

bool sampleMesh(const std::string& mesh_filename,
                const float k,
                const float output_resolution,
                pcl::PointCloud<pcl::PointNormal>& output_cloud)
{
  // Check if file exists
  if(!boost::filesystem::exists(mesh_filename))
  {
    ROS_ERROR("%s does not exist", mesh_filename.c_str());
    return false;
  }

  pcl::PolygonMesh polygon;
  pcl::io::loadPolygonFileSTL(mesh_filename, polygon);

  for(auto it = polygon.cloud.fields.begin(); it != polygon.cloud.fields.end(); ++it)
  {
    ROS_INFO("%s", it->name.c_str());
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::fromPCLPointCloud2(polygon.cloud, *cloud);

  pcl::CentroidPoint<pcl::PointXYZ> cp;
  for(auto pt = cloud->points.begin(); pt != cloud->points.end(); ++pt)
  {
    cp.add(*pt);
  }
  pcl::PointXYZ centroid;
  cp.get(centroid);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(k);

  // Set view point to be the centroid of the object such that all normals should be pointing inwards
  ne.setViewPoint(centroid.x, centroid.y, centroid.z);

  ne.compute(*normals);
  if(normals->points.size() == 0)
  {
    ROS_ERROR("Normal estimation resulted in point cloud with 0 normals");
    return false;
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud (new pcl::PointCloud<pcl::PointNormal> ());
  pcl::concatenateFields(*cloud, *normals, *normal_cloud);

  // Downsample point cloud to output resolution
  pcl::VoxelGrid<pcl::PointNormal> grid;
  grid.setInputCloud(normal_cloud);
  grid.setLeafSize(output_resolution, output_resolution, output_resolution);
  grid.filter(output_cloud);

  return true;
}

bool getSampledMesh(robot_reach_study::SampleMesh::Request& req,
                    robot_reach_study::SampleMesh::Response& res)
{
  // Check if a point cloud of the reach object has already been saved
  pcl::PointCloud<pcl::PointNormal> input_cloud;
  if(pcl::io::loadPCDFile(req.cloud_filename, input_cloud) == -1)
  {
    ROS_INFO("Generating new sampled point cloud...");

    // Change filename from package:// URI to full path name
    std::string mesh_filename = req.mesh_filename;
    if(!editMeshFilename(mesh_filename))
    {
      return false;
    }

    // Sample mesh into point cloud
    if(!sampleMesh(mesh_filename, req.n_neighbors, req.output_resolution, input_cloud))
    {
      return false;
    }

    // Save point cloud
    pcl::io::savePCDFileASCII(req.cloud_filename, input_cloud);
    ROS_INFO("New sampled point cloud saved");
  }
  else
  {
    ROS_INFO("Using previously saved reach object point cloud");
  }

  // Transform point cloud to correct frame
  tf::StampedTransform object_tf;
  if(!getObjectTF(req.world_name, req.object_name, object_tf))
  {
    return false;
  }

  Eigen::Affine3d transform;
  tf::transformTFToEigen(object_tf, transform);
  pcl::PointCloud<pcl::PointNormal> output_cloud;
  pcl::transformPointCloudWithNormals(input_cloud, output_cloud, transform.matrix());

  // Convert point cloud to message for output
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(output_cloud, msg);
  res.cloud = msg;

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
