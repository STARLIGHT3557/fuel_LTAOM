std_msgs/Header header
uav_mission_msgs/TaskContext context

uint8 START=1
uint8 STOP=2
uint8 HOLD_HOME=3
# Retry this return task without changing its batch or closing execution.
uint8 REPLAN=4
uint8 command

# Used by RETURN_HOME/HOLD_HOME. Coordinates are local ENU in header.frame_id.
geometry_msgs/Point goal
