#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <uav_mission_msgs/MissionCommand.h>
#include <uav_mission_msgs/PlannerStatus.h>
#include <uav_mission_msgs/TaskContext.h>
#include <uav_mission_msgs/TrajectoryStatus.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <stdexcept>
#include <thread>

using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::string;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;
struct LocalTrajData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, WAIT_EXEC };

class FastExplorationFSM {
private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;
  bool odom_twist_in_body_{false};
  bool return_home_{false}, active_trajectory_unsafe_{false};
  int no_frontier_cycles_{0}, plan_failures_{0};
  int finish_confirm_cycles_{3}, explore_max_failures_{3};
  double retry_interval_{1.0}, return_retry_interval_{1.0};
  double return_replan_period_{1.0}, home_tolerance_{0.20}, home_speed_tolerance_{0.15};
  ros::Time next_plan_attempt_, last_execution_status_, submitted_at_, terminal_error_since_;
  double handover_timeout_{2.0};
  shared_ptr<LocalTrajData> executing_traj_, submitted_traj_;
  int next_trajectory_id_{0};
  Vector3d return_goal_{0.0, 0.0, 1.0};
  uav_mission_msgs::TaskContext context_;
  uav_mission_msgs::TaskContext authorization_;
  uav_mission_msgs::TaskContext closed_context_;
  string source_id_{"fuel_planner"};
  uav_mission_msgs::MissionCommand last_mission_command_;
  bool have_mission_command_{false}, have_closed_context_{false};

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, stop_sub_, mission_sub_, authorization_sub_, trajectory_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_, planner_status_pub_;

  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const nav_msgs::PathConstPtr& msg);
  void missionCallback(const uav_mission_msgs::MissionCommandConstPtr& msg);
  void authorizationCallback(const uav_mission_msgs::TaskContextConstPtr& msg);
  void trajectoryCallback(const uav_mission_msgs::TrajectoryStatusConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void publishPlannerStatus(uint8_t state, const string& detail);
  bool validContext(const uav_mission_msgs::TaskContext& context) const;
  void visualize();
  void clearVisMarker();

public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif