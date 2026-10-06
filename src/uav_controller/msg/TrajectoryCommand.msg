# Controller -> trajectory publisher. Repeated commands with the same ID do not
# restart a trajectory. All positions and yaw use local ENU coordinates.
std_msgs/Header header
uav_mission_msgs/TaskContext context
uint64 trajectory_id
bool active
geometry_msgs/Point origin
float64 yaw
