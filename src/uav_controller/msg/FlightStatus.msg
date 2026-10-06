# Published on transitions/output-policy changes and at 10 Hz otherwise.
# previous_phase and transition_reason describe the most recent transition.
std_msgs/Header header
string mode
string mission_stage
string phase
string previous_phase
string transition_reason
uint64 transition_id
uav_mission_msgs/TaskContext context
uint64 trajectory_id
string output_policy
bool feedback_ready
string landing_state
float64 phase_elapsed
