# Immutable authorization identity for one task batch. A trajectory replan keeps
# this context and advances trajectory_id in the carrying message.
uint8 TASK_NONE=0
uint8 EXPLORE=1
uint8 RETURN_HOME=2

uint64 manager_epoch
uint64 mission_id
uint8 task_type
string source_id
