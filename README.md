# REACH
![](https://img.shields.io/badge/License-Apache%202.0-blue.svg)
[![Ubuntu](https://github.com/ros-industrial/reach/actions/workflows/ubuntu.yml/badge.svg)](https://github.com/ros-industrial/reach/actions/workflows/ubuntu.yml)

**R**obotic **E**valuation **A**nd **C**omparison **H**euristic

![Robot Reach Study][1]

![Reach Study Demo][2]

![Reach Study Heat Map][3]

## Table of Contents
- [Description](#Description)
- [Installation](#Installation)
- [ROS Integration](#ROS-Integration)
- [Tips](#Tips)

## Description
The REACH repository is a tool that allows users to visualize and quantitatively evaluate the reach capability of a robot system for a given workpiece.
See the ROSCon 2019 [presentation](docs/roscon2019_presentation.pdf) and [video](https://vimeo.com/378683038) for a more detailed explanation of the reach study concept and approach.

### Supported OS Distros
| OS            | Support |
| :---          | :---:   |
| Ubuntu Focal  | &check; |
| Ubuntu Jammy  | &check; |

### Structure
`reach` is a ROS-independent package that provides the framework for the reach study process, defined in the diagram below:

![Reach Study Flow Diagram][4]

The `reach` package also provides the interface definition for the required reach study functions:

1. [`TargetPoseGenerator`](include/reach/interfaces/target_pose_generator.h)
    - Generates Cartesian target poses that the robot should attempt to reach during the reach study
    - These target poses are expected to be relative to the kinematic base frame of the robot
    - The z-axis of the target poses is expected to oppose the z-axis of the robot kinematic tip frame
1. [`IKSolver`](include/reach/interfaces/ik_solver.h)
    - Calculates the inverse kinematics solution for the robot at an input 6 degree-of-freedom Cartesian target
1. [`Evaluator`](include/reach/interfaces/evaluator.h)
    - Calculates a numerical "fitness" score of an IK solution (i.e., robot joint pose) at a given Cartesian target pose
    - Higher values indicate better reachability
    - Example numerical measures of reachability include manipulability, distance from closest collision, etc.
1. [`Display`](include/reach/interfaces/display.h)
    - Visualizes the robot/reach study environment, target Cartesian poses, IK solutions, and reach study results
1. [`Logger`](include/reach/interfaces/logger.h)
    - Logs messages about the status and progress of the reach study

### Plugins
The interfaces described above are exposed as plugins using the [`boost_plugin_loader` library](https://github.com/tesseract-robotics/boost_plugin_loader) to support custom implementations.

Several default and dummy plugins have been created in the `reach` package.
Many other ROS-based plugins have been implemented in the [`reach_ros`](https://github.com/ros-industrial/reach_ros) and [`reach_ros2`](https://github.com/ros-industrial/reach_ros2) packages.
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
Nominally, the `reach` package is ROS-independent, but it is convenient to use the ROS dependency management and build tools to build the package.

First, clone the repository into a `catkin` workspace
``` bash
cd ~/reach_ws/src
git clone https://github.com/ros-industrial/reach.git
cd ..
```

Install the dependencies
``` bash
vcs import src < src/reach/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
```

Build the repository
```
<catkin/colcon> build
```

## ROS Integration
See the [`reach_ros`](https://github.com/ros-industrial/reach_ros) and [`reach_ros2`](https://github.com/ros-industrial/reach_ros2) repositories for ROS-based plugins, capability demos, and general usage instructions.

## Tips
1. Ensure the object mesh and reach target position scales match and are correct (visualize in `rviz`). It is common to be off by a factor of 1000.
1. If a set of robot links are allowed to collide with the mesh, add their names to the `touch_links` field of the `MoveItIKSolver` plugin in the reach study configuration file.
1. The selection of IK solver is key to the performance of the reach study. Gradient-based solvers (such as KDL and TRAC-IK) are typically good choices.
    - Additional constraints (or lack thereof, such as orientation freedom about the tool z-axis) can also be incorporated into the IK solver (via parameters or source code changes) to produce different reach study results
    - For `MoveIt`-based plugins, the selection of IK solver is defined in the `kinematics.yaml` file
1. Reach study results are serialized to file and can be loaded using the API in `reach` for programmatic analysis or modification
1. You can specify a starting seed for the IK solver by providing a list of joint positions in the config. This be used to help the IK solver to find solutions for complex scenarios. It might also help to guide the solver to solutions that are closer to a certain configuration that you prefere. If no initial seed is provided, a pose with all joints at 0 position is used.

[1]: docs/reach_study.png
[2]: docs/reach_study_demo.gif
[3]: docs/heat_map_colorized_mesh.png
[4]: docs/reach_study_flow_diagram.png
