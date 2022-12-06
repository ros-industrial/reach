# REACH
![](https://img.shields.io/badge/License-Apache%202.0-blue.svg)
[![Focal Noetic](https://github.com/ros-industrial/reach/actions/workflows/focal_noetic.yml/badge.svg)](https://github.com/ros-industrial/reach/actions/workflows/focal_noetic.yml)

**R**obotic **E**valuation **A**nd **C**omparison **H**euristic

**Table of Contents**
- [Description](#description)
- [Installation](#installation)
- [Demo](#demo)
- [Usage](#usage)
- [Hints](#hints)
- [Architecture and Interfaces](#architecture-and-interfaces)

![Robot Reach Study][1]

![Reach Study Demo][2]

![Reach Study Heat Map][3]

## Table of Contents
- [Description](#Description)
- [Installation](#Installation)
- [Demo](#Demo)
- [Usage](#Usage)
- [Tips](#Tips)

## Description
The REACH repository is a tool that allows users to visualize and quantitatively evaluate the reach capability of a robot system for a given workpiece.
See the ROSCon 2019 [presentation](docs/roscon2019_presentation.pdf) and [video](https://vimeo.com/378683038) for a more detailed explanation of the reach study concept and approach.

### Structure
`reach` is a ROS-independent package that provides the framework for the reach study process, defined in the diagram below:

![Reach Study Flow Diagram][4]

The `reach` package also provides the interface definition for the required reach study functions:

1. [`TargetPoseGenerator`](reach/include/reach/interfaces/target_pose_generator.h)
    - Generates Cartesian target poses that the robot should attempt to reach during the reach study
    - These target poses are expected to be relative to the kinematic base frame of the robot
    - The z-axis of the target poses is expected to oppose the z-axis of the robot kinematic tip frame
1. [`IKSolver`](reach/include/reach/interfaces/ik_solver.h)
    - Calculates the inverse kinematics solution for the robot at an input 6 degree-of-freedom Cartesian target
1. [`Evaluator`](reach/include/reach/interfaces/evaluator.h)
    - Calculates a numerical "fitness" score of an IK solution (i.e., robot joint pose) at a given Cartesian target pose
    - Higher values indicate better reachability
    - Example numerical measures of reachability include manipulability, distance from closest collision, etc.
1. [`Display`](reach/include/reach/interfaces/display.h)
    - Visualizes the robot/reach study environment, target Cartesian poses, IK solutions, and reach study results
1. [`Logger`](reach/include/reach/interfaces/logger.h)
    - Logs messages about the status and progress of the reach study

### Plugins
The interfaces described above are exposed as plugins using the [`boost_plugin_loader` library](https://github.com/tesseract-robotics/boost_plugin_loader) to support custom implementations.

Several default and dummy plugins have been created in the `reach` package.
Many other ROS1-based plugins have been implemented in the [`reach_ros`](reach_ros) package.
All of the plugins built in this project are discovered automatically by the plugin loader without additional manual steps.

The plugin loader class finds plugin libraries using two environment variables:
  - `LD_LIBRARY_PATH`
    - The plugin loader searches for libraries containing plugins within the directories defined in the `LD_LIBRARY_PATH` environment variable.
    - When using a ROS-based build tool such as `catkin` or `colcon` this variable is set automatically to include both system level and workspace level folders by sourcing `<devel|install>/setup.bash`
  - `REACH_PLUGINS`:
    - The plugin loader then looks for libraries with names defined by the environment variable `REACH_PLUGINS` within the directories specified by the environment variable `LD_LIBRARY_PATH`.
    - The names of these libraries should not include a prefix (e.g., `lib`) or a suffix (e.g., `.so`) and should be separated by a colon (`:`).
    - **This variable must be set manually to specify plugin libraries not built in this project**

If custom libraries created outside this project (for example `libmy_custom_reach_plugins.so` and `libcool_reach_plugins.so`) contain REACH plugins, make those plugin libraries visible to the plugin loader by setting the `REACH_PLUGINS` environment variable as follows:
``` bash
export REACH_PLUGINS=my_custom_reach_plugins:cool_reach_plugins
```

## Installation
Nominally, the `reach` package is ROS-independent, but it is convenient to use the ROS1 dependency management and build tools to build the package, as well as the `reach_ros` package.

First, clone the repository into a `catkin` workspace
``` bash
cd ~/catkin_ws/src
git clone https://github.com/ros-industrial/reach.git
cd ..
```

Install the dependencies
``` bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the repository
```
catkin build
```

## Demo
A simple demonstration of the capability of this repository is provided in the `reach_ros` package.
See the [instructions](reach_ros/demo/README.md) for details on how to run the demo.

## Usage
Use the following steps to run a reach study with a robot using the ROS1 infrastructure and plugins.

1. Create a URDF of your robot system
1. Create a launch file to load the URDF, SRDF, and other required parameters (e.g. related to kinematics, joint, limits) to the parameter server (see [this demo example file](reach_ros/demo/config/robot.launch))
1. Create a mesh model of the workpiece
    > Note: the origin of this model should align with the kinematic base frame of the robot
1. Create a point cloud of the target points on the workpiece
    - This point cloud can be generated using a command line tool from PCL 1.8:
      ```
      pcl_mesh_sampling <workpiece_mesh>.ply <output_cloud>.pcd -n_samples <number of samples> -leaf_size <leaf_size> -write_normals true
      ```
1. Create a configuration YAML file defining the parameters of the reach study and the configuration of the interface plugins (see [this demo example](reach_ros/demo/config/reach_study.yaml))
1. Run the setup launch file
    ```
    roslaunch reach_ros setup.launch robot:=<load_robot_parameters>.launch
    ```
1. Run the reach study analysis
    ```
    roslaunch reach_ros start.launch config_file:=<config_file.yaml> config_name:=<arbitrary_config>
    ```

## Tips
1. Ensure the object mesh and reach target position scales match and are correct (visualize in `rviz`). It is common to be off by a factor of 1000.
1. If a set of robot links are allowed to collide with the mesh, add their names to the `touch_links` field of the `MoveItIKSolver` plugin in the reach study configuration file.
1. The selection of IK solver is key to the performance of the reach study. Gradient-based solvers (such as KDL and TRAC-IK) are typically good choices.
    - Additional constraints (or lack thereof, such as orientation freedom about the tool z-axis) can also be incorporated into the IK solver (via parameters or source code changes) to produce different reach study results
    - For `MoveIt`-based plugins, the selection of IK solver is defined in the `kinematics.yaml` file
1. Reach study results are serialized to file and can be loaded using the API in `reach` for programmatic analysis or modification

[1]: docs/reach_study.png
[2]: reach_ros/demo/docs/reach_study_demo.gif
[3]: docs/heat_map_colorized_mesh.png
[4]: docs/reach_study_flow_diagram.png
