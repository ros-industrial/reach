#include <ros/ros.h>
#include <ros/package.h>
#include <robot_reach_study/SampleMesh.h>
#include <urdf/model.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <vtk_viewer/vtk_utils.h>
#include <vtk_viewer/vtk_viewer.h>
#include <tool_path_planner/raster_tool_path_planner.h>

boost::optional<tf::StampedTransform> getObjectTF(const std::string& world_frame,
                                                  const std::string& object_frame)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;

  if(listener.waitForTransform(world_frame, object_frame, ros::Time::now(), ros::Duration(5.0)))
  {
    try
    {
      listener.lookupTransform(world_frame, object_frame, ros::Time(0.0), transform);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return {};
    }
  }
  else
  {
    ROS_ERROR("TF lookup between %s and %s has timed out", world_frame.c_str(), object_frame.c_str());
    return {};
  }
  boost::optional<tf::StampedTransform> result = transform;
  return {result};
}

bool editMeshFilename(std::string& filename)
{
  // Define the word to search for
  const std::string prefix = "package://";

  // Find the prefix to remove in the current filename
  size_t pos = filename.find(prefix);
  if(pos != std::string::npos)
  {
    // Erase prefix from filename
    filename.erase(pos, prefix.length());

    // Find the first "/" in the filename (indicating end of package name)
    size_t start_pos = filename.find_first_of("/", 0);

    // Create package name string and get the full package name file path
    std::string pkg_name = filename.substr(pos, start_pos);
    std::string pkg_path = ros::package::getPath(pkg_name);
    if(pkg_path.length() == 0)
    {
      // Save the iterator to the filename if the ROS package where it came from is not found
      ROS_FATAL("Package not found: %s", pkg_name.c_str());
      return false;
    }

    // Erase the package name from the front of the filename and replace with full package path
    filename.erase(0, start_pos);
    filename.insert(filename.begin(), pkg_path.begin(), pkg_path.end());
  }
  else
  {
    ROS_FATAL("Mesh filename not in correct format: %s", filename.c_str());
    return false;
  }
  return true;
}

pcl::PointCloud<pcl::PointNormal> sampleMesh(const std::string& mesh_filename,
                                             const float sampling_resolution,
                                             const float output_resolution)
{
  // Check if file exists

  // Load STL and generate normals if PolyData object was filled
  vtkSmartPointer<vtkPolyData> poly = vtk_viewer::readSTLFile(mesh_filename);
  vtk_viewer::generateNormals(poly, 0);

  // Sample the mesh
  vtkSmartPointer<vtkPolyData> sampled_poly = vtk_viewer::sampleMesh(poly, sampling_resolution);

  // Create a tool path planner and create normals for the sampled mesh
  tool_path_planner::RasterToolPathPlanner planner;
  tool_path_planner::ProcessTool tool;
  tool.nearest_neighbors = 6;
  planner.setTool(tool);
  planner.setInputMesh(poly);
  planner.estimateNewNormals(sampled_poly);

  // Show sampled mesh in viewer
  vtk_viewer::VTKViewer viz;
  std::vector<float> color(3);
  color[0] = 0.0;
  color[1] = 0.0;
  color[2] = 1.0;
  viz.addPolyNormalsDisplay(sampled_poly, color, 0.05f);
  viz.renderDisplay();

  // Convert VTK PolyData to PCL point cloud
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal> ());
  vtk_viewer::VTKtoPCL(sampled_poly, *cloud);

  // Downsample point cloud to output resolution
  pcl::VoxelGrid<pcl::PointNormal> grid;
  grid.setInputCloud(cloud);
  grid.setLeafSize(output_resolution, output_resolution, output_resolution);
  pcl::PointCloud<pcl::PointNormal> output_cloud;
  grid.filter(output_cloud);

  return output_cloud;
}

bool getSampledMesh(robot_reach_study::SampleMesh::Request& req,
                    robot_reach_study::SampleMesh::Response& res)
{
  // Attempt to load existing point cloud
  pcl::PointCloud<pcl::PointNormal> input_cloud;
  if(pcl::io::loadPCDFile(req.cloud_filename, input_cloud) == -1)
  {
    ROS_INFO("Sampled point cloud for reach object not found");
    ROS_INFO("Generating new sampled point cloud...");

    // Change filename from package:// URI to full path name
    std::string mesh_filename = req.mesh_filename;
    if(!editMeshFilename(mesh_filename))
       return false;

    // Sample mesh into point cloud
    input_cloud = sampleMesh(mesh_filename, req.sampling_resolution, req.output_resolution);

    // Save point cloud
    pcl::io::savePCDFileASCII(req.cloud_filename, input_cloud);
    ROS_INFO("New sampled point cloud saved");
  }

  // Transform point cloud to correct frame
  boost::optional<tf::StampedTransform> object_tf = getObjectTF(req.world_name, req.object_name);
  if(!object_tf)
    return false;

  Eigen::Affine3d transform;
  tf::transformTFToEigen(*object_tf, transform);
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
  ros::NodeHandle pnh;

  // Create a server
  ros::ServiceServer service = pnh.advertiseService("/sample_mesh", getSampledMesh);

  ros::spin();

  return 0;
}
