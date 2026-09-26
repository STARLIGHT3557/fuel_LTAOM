std_msgs/Header header
uav_mission_msgs/TaskContext context
uint64 trajectory_id

uint8 PENDING=1
uint8 ACTIVE=2
uint8 REJECTED=3
uint8 COMPLETED=4
uint8 INVALID=5
uint8 state
string detail
