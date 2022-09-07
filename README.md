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

![Reach Study Heat Map][2]

## Description

The REACH repository is a tool that allows users to visualize and quantitatively evaluate the reach capability of a robot system for a given workpiece.
See the ROSCon 2019 [presentation](docs/roscon2019_presentation.pdf) and [video](https://vimeo.com/378683038) for a more detailed explanation of the reach study concept and approach.

The framework for the reach study process is outlined in the diagram below:

![Reach Study Flow Diagram][3]

## Installation

```
cd ~/catkin_ws/src
git clone https://github.com/ros-industrial/reach.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

## Demo

A simple demonstration of the capability of this repository is provided in the `reach_demo` package.
See the [instructions](reach_demo/README.md) for details on how to run the demo.

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

The algorithm searches for alignment of the TCP Z-axis with the pointcloud normals.

## Hints

1. Ensure the object mesh scale and the point cloud scale match and are correct in RViz. It is common to be off by a factor of 1000.
1. If it is OK for a robot link to collide with the mesh, add the link to "touch_links" fields in the config file.
1. A different IK solver may yield better results than the default. A good choice is TracIK. Typically this is configured in kinematics.yaml.
1. reach_core has some options for programmatically querying the reachability database.

## Architecture and Interfaces

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

The REACH core package also provides the interface definition for the required reach study functions:

1. Robot Pose Evaluator
    - Calculates a numerical score for an input robot pose
    - Example numerical measures of reachability
      - Robot manipulability
      - Distance from closest collision
1. Inverse Kinematics Solver
    - Calculates the inverse kinematics solution for the robot at an input 6 degree-of-freedom Cartesian target
    - Contains an evaluator interface to assign a value to the resulting IK solution
1. Reach Display
    - Provides interactive markers for the target positions to display reachability status and visualize the robot goal and seed poses at those targets

These interfaces can be implemented as custom plugins to provide application-specific behavior.
Provided plugin implementations can be found in the [`moveit_reach_plugins` package](moveit_reach_plugins).

[1]: docs/reach_study.png
[2]: docs/heat_map_colorized_mesh.png
[3]: docs/reach_study_flow_diagram.png
