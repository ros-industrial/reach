# An individual reach point
string id
geometry_msgs/Pose goal
bool reached
sensor_msgs/JointState goal_state
sensor_msgs/JointState seed_state
float64 score
# needed for path computation
bool retrieved
# in packets of size goal_state.name.size()
float64[] joint_space_trajectory
# each has x,y,z qx,qy,qz,qw
float64[] waypoints
# ik solver name
string ik_solver
