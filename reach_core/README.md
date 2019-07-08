# REACH Core

## Approach

The REACH core package provides the framework for the reach study process, defined in the diagram below:

![Reach Study Flow Diagram][1]

## Interfaces

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

[1]: docs/reach_study_flow_diagram.png
