# Trajectory publisher -> controller. Full task context and ID reject stale data.
std_msgs/Header header
uav_mission_msgs/TaskContext context
uint64 trajectory_id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float64 yaw
bool finished
