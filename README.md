# REACH
![](https://travis-ci.com/ros-industrial/reach.svg?branch=master)
![](https://img.shields.io/badge/License-Apache%202.0-blue.svg)

**R**obotic **E**valuation **A**nd **C**omparison **H**euristic

![Robot Reach Study][1]

![Reach Study Heat Map][2]

## Description

The REACH repository is a tool that allows users to visualize and quantitatively evaluate the reach capability of
a robot system for a given workpiece.

The package is comprised of several packages:
- `reach_msgs`
  - Message definitions for the reach study
- `reach_core`
  - Core code to operate the reach study
  - Interfaces for plugins
    - Inverse kinematics solver
    - Robot pose evaluator
    - Reach Display
- `moveit_reach_plugins`
  - Implementations of the plugin interfaces built on the MoveIt! planning framework

## Installation

```
cd ~/colcon_ws/src
git clone https://github.com/ros-industrial/reach.git -b ros2
cd ..
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
colcon build --symlink-install --mixin rel-with-deb-info compile-commands ccache
```

## Demo

A simple demonstration of the capability of this repository is provided in the `reach_demo` package. See the [instructions](reach_demo/README.md) for details on how to run the demo.

## Usage

1. Create a URDF of your robot system
1. Create a launch file to load the URDF, SRDF, and other required parameters (e.g. related to kinematics, joint, limits) to the parameter server
1. Create a mesh model of the workpiece
1. Create a point cloud of the target points on the workpiece
    - This point cloud can be generated using a command line tool from PCL 1.8:
      ```
      pcl_mesh_sampling <workpiece_mesh>.ply <output_cloud>.pcd -n_samples <number of samples> -leaf_size <leaf_size> -write_normals true
      ```
1. Create a configuration YAML file (see example in config directory)
1. Run the setup launch file
    ```
    roslaunch reach_core setup.launch robot:=<load_robot_parameters>.launch
    ```
1. Run the reach study analysis
    ```
    roslaunch reach_core start.launch config_file:=<config_file.yaml> config_name:=<arbitrary_config>
    ```

[1]: reach_core/docs/reach_study.png
[2]: reach_core/docs/heat_map_colorized_mesh.png
