std_msgs/Header header
uav_mission_msgs/TaskContext context

uint8 READY=0
uint8 EXPLORING=1
uint8 PREPARE_RETURN=2
uint8 RETURNING=3
uint8 HOME_HOLD=4
uint8 RETURN_RECOVERY=5
uint8 state

uint8 REASON_NONE=0
uint8 EXPLORATION_DONE=1
uint8 LOW_BATTERY=2
uint8 EXPLORE_PLAN_FAILED=3
uint8 EXPLORE_PLAN_TIMEOUT=4
uint8 TRAJECTORY_TIMEOUT=5
uint8 MANUAL_RETURN=6
uint8 STOP_REQUESTED=7
uint8 RETURN_PLAN_FAILED=8
uint8 UNSAFE_TRAJECTORY=9
uint8 return_reason

# Preserve the original return trigger while reporting recovery separately.
uint8 recovery_reason
uint32 recovery_attempts

bool battery_valid
float32 battery_percentage
bool mission_done
string detail
