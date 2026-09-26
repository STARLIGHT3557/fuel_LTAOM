std_msgs/Header header
uav_mission_msgs/TaskContext context
uint64 trajectory_id

uint8 IDLE=0
uint8 PLANNING=1
uint8 TRAJECTORY_READY=2
uint8 EXECUTING=3
uint8 EXPLORATION_DONE=4
uint8 GOAL_REACHED=5
uint8 FAILED=6
uint8 OUTPUT_TIMEOUT=7
uint8 TRAJECTORY_UNSAFE=8
uint8 state

uint32 consecutive_failures
string detail
