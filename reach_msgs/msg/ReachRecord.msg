# An individual reach point
string id
geometry_msgs/Pose goal
bool reached
sensor_msgs/JointState goal_state
sensor_msgs/JointState seed_state
float64 score
# needed for path computation
bool retrieved
sensor_msgs/JointState[] joint_space_trajectory
geometry_msgs/Pose[] waypoints
