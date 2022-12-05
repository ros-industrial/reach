# REACH ROS Plugins

This package contains the ROS1-based plugin implemenations of REACH kinematics, evaluation, and display interfaces

## Evaluation Plugins

### Manipulability

This plugin uses MoveIt! to calculate the manipulability of a robot pose. Higher manipulability results in a higher pose score. Range: [0, inf)

Parameters:

- **`planning_group`**
  - The name of the planning group with which to evaluate the manipulability of a given robot pose
- **`jacobian_row_subset`** (optional)
  - The indices of the rows of the Jacobian to use when evaluating the manipulability. The row indices should be on [0, 6) and correspond to the output space [x, y, z, rx, ry, rz]
  - Ex. `jacobian_row_subset: [0, 1, 2]  # position manipulability only`

### Manipulability Scaled

This plugin uses MoveIt! to calculate the manipulability of a robot pose divided by the characteristic length of the motion group.
The characteristic length is computed by walking from the base link to the tip link of the motion group and summing the distances between adjacent links.
Higher scaled manipulability results in a higher pose score. Range: [0, inf)

Parameters:

- **`planning_group`**
  - The name of the planning group with which to evaluate the manipulability of a given robot pose
- **`jacobian_row_subset`** (optional)
  - The indices of the rows of the Jacobian to use when evaluating the manipulability. The row indices should be on [0, 6) and correspond to the output space [x, y, z, rx, ry, rz]
  - Ex. `jacobian_row_subset: [0, 1, 2]  # position manipulability only`
- **`excluded_links`** (optional)
  - The names of links contained in the motion group that should not contribute to the characteristic length

### Manipulability Ratio

This plugin uses MoveIt! to calculate the manipulability of a robot pose and evaluate a score. The score is calculated as the ratio of the smallest manipulability value to the largest manipulability value.
The larger this ratio, the more uniform the dexterity and the higher the score. Range [0, 1]

Parameters:

- **`planning_group`**
  - The name of the planning group with which to evaluate the manipulability of a given robot pose
- **`jacobian_row_subset`** (optional)
  - The indices of the rows of the Jacobian to use when evaluating the manipulability. The row indices should be on [0, 6) and correspond to the output space [x, y, z, rx, ry, rz]
  - Ex. `jacobian_row_subset: [0, 1, 2]  # position manipulability only`

### Distance Penalty

This plugin uses the MoveIt! collision environment to calculate the distance to closest collision
for a robot pose. That distance value is then used to score the robot pose. Larger distance to closest collision
results in higher pose score. Range: [0, inf)

Parameters:

- **`planning_group`**
  - The name of the planning group to be used to solve the robot's inverse kinematics
- **`distance_threshold`**
  - The distance between 2 closest surfaces to collision under which an inverse kinematics solution will be considered invalid
- **`collision_mesh_filename`**
  - The filename (in ROS package URI format) of the reach object mesh to be used to do collision checking
  - Example: `package://<your_package>/<folder>/<filename>.stl
- **`touch_links`**
  - The names of the robot links with which the reach object mesh is allowed to collide
- **`exponent`**
  - score = (closest_distance_to_collision - distance_threshold)^exponent.

### Joint Penalty

This plugin uses the MoveIt! robot model to calculate a robot pose score based on how much the pose deviates
from the center of the joint range. Robot poses that are closer to the center of the joint range result in higher pose scores. Range: [0, 1]

Parameters:

- **`planning_group`**
  - The name of the planning_group with which to evaluate the joint penalty

## IK Solvers

### MoveIt! IK Solver

This plugin uses MoveIt! kinematics solvers and collision checking to produce collision aware IK solutions

Parameters:

- **`planning_group`**
  - Name of the planning group
- **`distance_threshold`**
  - The distance from nearest collision at which to invalidate an IK solution. For example, if this parameter is
  set to 0.1m, then IK solutions whose distance to nearest collision is less than 0.1m will be invalidated
- **`collision_mesh_filename`**
  - The file path to the collision mesh model of the workpiece, in the `package://` or 'file://' URI format
- **`touch_links`**
  - The TF links that are allowed to be in contact with the collision mesh
- **`evaluation_plugin`**
  - The name (and parameters) of the evaluation plugin to be used to score IK solution poses

### Discretized MoveIt! IK Solver

This plugin performs the same function as the MoveIt! IK solver plugin above, but calculates IK solutions for
a target that has been discretized about its Z-axis by an input angle. The pose with the best score is returned.

Parameters:

- **`planning_group`**
  - Name of the planning group
- **`distance_threshold`**
  - The distance from nearest collision at which to invalidate an IK solution. For example, if this parameter is
  set to 0.1m, then IK solutions whose distance to nearest collision is less than 0.1m will be invalidated
- **`collision_mesh_filename`**
  - The file path to the collision mesh model of the workpiece, in the `package://` or 'file://' URI format
- **`touch_links`**
  - The TF links that are allowed to be in contact with the collision mesh
- **`evaluation_plugin`**
  - The name (and parameters) of the evaluation plugin to be used to score IK solution poses
- **`discretization_angle`**
  - The angle (between 0 and pi, in radians) with which to sample each target pose about the Z-axis

## Display Plugins

### ROS Reach Display

This plugin publishes joint state and visualization markers to display the results of the reach study.
The markers are interactive:
  - Left-clicking on a marker will change the robot position to the IK solution
    - This works by publishing a `sensor_msgs/JointState` message to the `reach_joints` topic, which is a source topic for the `joint_state_publisher`
  - Right-clicking on the marker will bring up a context menu that displays the score of the reach target

The markers are colorized with a heat-map, where red represents the highest scores (i.e., hottest) and blue represents the lowest scores (i.e., coldest).
Unreachable points are colorized black. There are two methods for computing the values for the heat map:
  1. By default, the values for the heat map are computed as the ratio of the individual scores to the maximum score.
  Thus if the scores of the reach study are all fairly consistent and close to the maximum, then the colors of the markers will be mostly red (with the target with the maximum score being the deepest hue of red).
  If the scores of the reach study targets differ greatly, more variation in color will be seen.
  Usually this method produces the most meaningful display.
  1. The values can also be computed using the "full color range", where the scores are fully normalized before colorization.
  The lowest score (regardless of value) is always displayed as the deepest hue of blue and the the highest score is always shown as the deepest hue of red.
  This is valuable for highlighting differences in reachability but can be misleading due to the normalization of the scores.

Parameters:

- **`collision_mesh_filename`**
  - The file path to the collision mesh model of the workpiece, in the `package://` or 'file://' URI format
- **`kinematic_base_frame`**
  - The base frame of the kinematic tree in which to display the interactive markers
- **`marker_scale`**
  - The length (in meters) of the arrow markers representing the target Cartesian points
- **`use_full_color_range`** (optional, default: False)
  - Colorize the heat map using the full range of colors (such that the target with the lowest score is the deepest hue of blue, and the target with the highest score is the deepest hue of red)
