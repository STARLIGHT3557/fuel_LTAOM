#include "uav_controller.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

using Command = prometheus_msgs::UAVCommand;
using Control = prometheus_msgs::UAVControlState;

UAV_controller::UAV_controller(ros::NodeHandle& nh, const Eigen::Vector3d& kp, const Eigen::Vector3d& kv)
    : k_pos_(kp), k_vel_(kv) {
  nh.param("uav_id", uav_id_, 1);
  nh.param("control/hover_percentage", hover_percentage, 0.258);
  nh.param("control/state_timeout", state_timeout_, 0.5);
  nh.param("control/command_timeout", command_timeout_, 0.5);
  const char* axes[] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    nh.param(std::string("geo_fence/") + axes[i] + "_min", fence_min_[i], i == 2 ? -50.0 : -1000.0);
    nh.param(std::string("geo_fence/") + axes[i] + "_max", fence_max_[i], 1000.0);
  }
  if (!(hover_percentage > 0 && hover_percentage < 1) || !k_pos_.allFinite() || !k_vel_.allFinite() ||
      state_timeout_ <= 0 || command_timeout_ <= 0 || (fence_min_.array() >= fence_max_.array()).any())
    throw std::runtime_error("Invalid P600 controller parameters");
  const std::string ns = "/uav" + std::to_string(uav_id_);
  command_sub_ = nh.subscribe(ns + "/prometheus/command", 1, &UAV_controller::commandCallback, this);
  state_sub_ = nh.subscribe(ns + "/prometheus/state", 1, &UAV_controller::stateCallback, this);
  px4_setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>(ns + "/mavros/setpoint_raw/local", 1);
  px4_setpoint_raw_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>(ns + "/mavros/setpoint_raw/attitude", 1);
  control_pub_ = nh.advertise<Control>(ns + "/prometheus/control_state", 1);
  mode_client_ = nh.serviceClient<mavros_msgs::SetMode>(ns + "/mavros/set_mode");
  command_.Agent_CMD = Command::Current_Pos_Hover;
  resetThrustMapping();
  ROS_INFO("P600 controller: mode 9 attitude/thrust tracking, mode 0 position takeoff/hold");
}

void UAV_controller::hold() {
  hold_position_ = position_;
  hold_yaw_ = yaw_;
  have_hold_ = true;
  command_.Agent_CMD = Command::Current_Pos_Hover;
}

void UAV_controller::stateCallback(const prometheus_msgs::UAVState::ConstPtr& msg) {
  const Eigen::Vector3d pos(msg->position[0], msg->position[1], msg->position[2]);
  const Eigen::Vector3d vel(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
  const Eigen::Quaterniond q(msg->attitude_q.w, msg->attitude_q.x, msg->attitude_q.y, msg->attitude_q.z);
  if (!pos.allFinite() || !vel.allFinite() || !q.coeffs().allFinite() || std::abs(q.norm() - 1) > 0.01 ||
      !std::isfinite(msg->a[0]) || !std::isfinite(msg->a[1]) || !std::isfinite(msg->a[2])) return;
  uav_state = *msg;
  position_ = pos;
  velocity_ = vel;
  attitude_ = q;
  yaw_ = std::atan2(2 * (q.w()*q.z() + q.x()*q.y()), 1 - 2 * (q.y()*q.y() + q.z()*q.z()));
  have_state_ = true;
  state_time_ = ros::Time::now();
  const bool valid = msg->connected && msg->odom_valid;
  const bool offboard = valid && msg->armed && msg->mode == "OFFBOARD";
  // Before OFFBOARD, stream the measured pose rather than an old ground/exit
  // target. On entry, latch the current pose once; while active, keep that
  // target fixed until a new command arrives. Handle the edge here so a Move
  // received after this state update is not overwritten by the main loop.
  if (valid && (!have_hold_ || !offboard || !was_offboard_)) hold();
  was_offboard_ = offboard;
}

void UAV_controller::commandCallback(const Command::ConstPtr& msg) {
  if (!have_state_ || !uav_state.connected || !uav_state.odom_valid ||
      (ros::Time::now() - state_time_).toSec() > state_timeout_ || landing_) return;
  if (command_.Control_Level == Command::ABSOLUTE_CONTROL && msg->Control_Level == Command::DEFAULT_CONTROL) return;
  if (msg->Agent_CMD == Command::Land) {
    landing_ = true;
    return;
  }
  if (msg->Agent_CMD == Command::Current_Pos_Hover || msg->Agent_CMD == Command::Init_Pos_Hover) {
    hold();
    command_.Control_Level = msg->Control_Level;
    return;
  }
  if (msg->Agent_CMD != Command::Move || msg->Yaw_Rate_Mode ||
      (msg->Move_mode != Command::XYZ_POS && msg->Move_mode != Command::TRAJECTORY_ATTITUDE)) {
    ROS_WARN_THROTTLE(2, "P600 accepts XYZ_POS (0), TRAJECTORY_ATTITUDE (9), hold and land only");
    return;
  }
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(msg->position_ref[i]) || !std::isfinite(msg->velocity_ref[i]) ||
        !std::isfinite(msg->acceleration_ref[i]) || msg->position_ref[i] < fence_min_[i] ||
        msg->position_ref[i] > fence_max_[i]) return;
  }
  if (!std::isfinite(msg->yaw_ref)) return;
  // Do not queue position or trajectory targets while another PX4 mode owns
  // the aircraft. A fresh Move command is required after entering OFFBOARD.
  if (!was_offboard_) {
    ROS_WARN_THROTTLE(2, "Move rejected: arm and enter OFFBOARD before sending a target");
    return;
  }
  command_ = *msg;
  command_time_ = ros::Time::now();
}

void UAV_controller::mainloop() {
  const auto now = ros::Time::now();
  const bool connected = have_state_ && uav_state.connected && (now - state_time_).toSec() <= state_timeout_;
  const bool valid = connected && uav_state.odom_valid;
  const bool in_fence = (position_.array() >= fence_min_.array()).all() && (position_.array() <= fence_max_.array()).all();
  const bool offboard = valid && uav_state.armed && uav_state.mode == "OFFBOARD";
  if (connected && !uav_state.armed) {
    if (landing_) hold();
    landing_ = false;
    failsafe_ = false;
  } else if (have_state_ && uav_state.armed && (!valid || !in_fence)) {
    landing_ = true;
    failsafe_ = true;
  }
  // Mode transitions and the hover latch are updated in stateCallback().
  if (!offboard) resetThrustMapping();
  else estimateThrustStopFlag = false;

  Control status;
  status.header.stamp = now;
  status.uav_id = uav_id_;
  status.pos_controller = Control::PX4_ORIGIN;  // Retained wire value; mode 9 uses the controller below.
  status.control_state = landing_ ? Control::LAND_CONTROL : (offboard ? Control::COMMAND_CONTROL : Control::INIT);
  status.failsafe = failsafe_ || !valid;
  control_pub_.publish(status);
  if (!connected) return;  // Stop setpoints; PX4 handles its own offboard-link loss.
  if (landing_) {
    if (uav_state.armed && uav_state.mode != "AUTO.LAND" && (now - mode_request_time_).toSec() >= 1) {
      mavros_msgs::SetMode srv;
      srv.request.custom_mode = "AUTO.LAND";
      mode_request_time_ = now;
      if (!mode_client_.call(srv) || !srv.response.mode_sent) ROS_WARN_THROTTLE(2, "AUTO.LAND request failed; retrying");
    }
    return;
  }
  if (!valid) return;
  if (command_.Agent_CMD == Command::Move && command_.Move_mode == Command::TRAJECTORY_ATTITUDE &&
      (now - command_time_).toSec() > command_timeout_) {
    ROS_WARN("Trajectory timed out; latching current position for hover");
    hold();
  }
  if (command_.Agent_CMD != Command::Move) {
    send_pos_setpoint(hold_position_, hold_yaw_);
  } else {
    const Eigen::Vector3d pos(command_.position_ref[0], command_.position_ref[1], command_.position_ref[2]);
    if (command_.Move_mode == Command::XYZ_POS) send_pos_setpoint(pos, command_.yaw_ref);
    else if (offboard) {
      const Eigen::Vector3d vel(command_.velocity_ref[0], command_.velocity_ref[1], command_.velocity_ref[2]);
      const Eigen::Vector3d acc(command_.acceleration_ref[0], command_.acceleration_ref[1], command_.acceleration_ref[2]);
      send_att_thrust_setpoint(pos, vel, acc, command_.yaw_ref, position_, velocity_, yaw_, attitude_);
    }
  }
}

void UAV_controller::resetThrustMapping(void)
{
  thr2acc_ = 9.81 / hover_percentage;
  P_ = 1e6;
  estimateThrustStopFlag = true;
  while (!timed_thrust_.empty()) timed_thrust_.pop();
}


void UAV_controller::send_pos_setpoint(const Eigen::Vector3d &pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.header.stamp = ros::Time::now();
    pos_setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                             mavros_msgs::PositionTarget::IGNORE_VY |
                             mavros_msgs::PositionTarget::IGNORE_VZ |
                             mavros_msgs::PositionTarget::IGNORE_AFX |
                             mavros_msgs::PositionTarget::IGNORE_AFY |
                             mavros_msgs::PositionTarget::IGNORE_AFZ |
                             mavros_msgs::PositionTarget::IGNORE_AFY |
                             mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.yaw = yaw_sp;
    px4_setpoint_raw_local_pub.publish(pos_setpoint);
}

Eigen::Matrix3d UAV_controller::quat2RotMatrix(const Eigen::Vector4f &q){
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3),
    2 * q(1) * q(2) - 2 * q(0) * q(3),
    2 * q(0) * q(2) + 2 * q(1) * q(3),

    2 * q(0) * q(3) + 2 * q(1) * q(2),
    q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
    2 * q(2) * q(3) - 2 * q(0) * q(1),

    2 * q(1) * q(3) - 2 * q(0) * q(2),
    2 * q(0) * q(1) + 2 * q(2) * q(3),
    q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}

Eigen::Vector4f UAV_controller::rot2Quaternion(const Eigen::Matrix3d &Rotmatrix){
  Eigen::Vector4f quat;
  double tr = Rotmatrix.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (Rotmatrix(2, 1) - Rotmatrix(1, 2)) / S;
    quat(2) = (Rotmatrix(0, 2) - Rotmatrix(2, 0)) / S;
    quat(3) = (Rotmatrix(1, 0) - Rotmatrix(0, 1)) / S;
  } else if ((Rotmatrix(0, 0) > Rotmatrix(1, 1)) & (Rotmatrix(0, 0) > Rotmatrix(2, 2))) {
    double S = sqrt(1.0 + Rotmatrix(0, 0) - Rotmatrix(1, 1) - Rotmatrix(2, 2)) * 2.0; // S=4*qx
    quat(0) = (Rotmatrix(2, 1) - Rotmatrix(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (Rotmatrix(0, 1) + Rotmatrix(1, 0)) / S;
    quat(3) = (Rotmatrix(0, 2) + Rotmatrix(2, 0)) / S;
  } else if (Rotmatrix(1, 1) > Rotmatrix(2, 2)) {
    double S = sqrt(1.0 + Rotmatrix(1, 1) - Rotmatrix(0, 0) - Rotmatrix(2, 2)) * 2.0; // S=4*qy
    quat(0) = (Rotmatrix(0, 2) - Rotmatrix(2, 0)) / S;
    quat(1) = (Rotmatrix(0, 1) + Rotmatrix(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (Rotmatrix(1, 2) + Rotmatrix(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + Rotmatrix(2, 2) - Rotmatrix(0, 0) - Rotmatrix(1, 1)) * 2.0; // S=4*qz
    quat(0) = (Rotmatrix(1, 0) - Rotmatrix(0, 1)) / S;
    quat(1) = (Rotmatrix(0, 2) + Rotmatrix(2, 0)) / S;
    quat(2) = (Rotmatrix(1, 2) + Rotmatrix(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}


bool UAV_controller::estimateThrustModel(const Eigen::Vector3d &est_a)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();

    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;

    return true;
  }
  return false;
}


#define g 9.81
void UAV_controller::send_att_thrust_setpoint(const Eigen::Vector3d &pos_sp, const Eigen::Vector3d &vel_sp, const Eigen::Vector3d &acc_des ,float yaw_sp,
                                        const Eigen::Vector3d &uav_pos, const Eigen::Vector3d &uav_vel, const float &uav_yaw,
                                        const Eigen::Quaterniond &uav_quat)
{
    Eigen::Vector3d  pos_error = pos_sp - uav_pos;
    Eigen::Vector3d  vel_error = vel_sp - uav_vel;

    Eigen::Vector3d a_fb = k_pos_.asDiagonal()* pos_error + k_vel_.asDiagonal()* vel_error;

    if(a_fb.norm() > 8)  a_fb = (8 / a_fb.norm()) * a_fb;

    const Eigen::Vector3d a_des = a_fb + g*Eigen::Vector3d(0,0,1) + acc_des;

    Eigen::Vector4f quat_current;
    quat_current(0) = static_cast<float>(uav_quat.w());  // 注意加括号()
    quat_current(1) = static_cast<float>(uav_quat.x());
    quat_current(2) = static_cast<float>(uav_quat.y());
    quat_current(3) = static_cast<float>(uav_quat.z());

    Eigen::Matrix3d rotation_current = quat2RotMatrix(quat_current);
    Eigen::Vector3d uav_esti_acc;
    uav_esti_acc << uav_state.a[0], uav_state.a[1], uav_state.a[2];
    if(!estimateThrustStopFlag) estimateThrustModel(uav_esti_acc);
    double thrust = a_des.dot(rotation_current*Eigen::Vector3d(0,0,1)) / thr2acc_;  // px_ctrl 高度融合


    float psi = yaw_sp;
    Eigen::Vector3d proj_xb_des(cosf(psi),sinf(psi),0);
    Eigen::Vector3d zb_des = a_des/a_des.norm();
    Eigen::Vector3d yb_des = zb_des.cross(proj_xb_des)/(zb_des.cross(proj_xb_des).norm());
    Eigen::Vector3d xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();
    Eigen::Matrix3d R_des;
    R_des << xb_des(0), yb_des(0), zb_des(0),
            xb_des(1), yb_des(1), zb_des(1),
            xb_des(2), yb_des(2), zb_des(2);

    Eigen::Vector4f quat_des = rot2Quaternion(R_des);

    mavros_msgs::AttitudeTarget local_attitude_target;
    local_attitude_target.header.stamp = ros::Time::now();
    local_attitude_target.orientation.w = quat_des(0);
    local_attitude_target.orientation.x = quat_des(1);
    local_attitude_target.orientation.y = quat_des(2);
    local_attitude_target.orientation.z = quat_des(3);
    local_attitude_target.thrust = std::max(0.0,std::min(1.0, thrust+0.01));
    local_attitude_target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE| mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    px4_setpoint_raw_attitude_pub.publish(local_attitude_target);                      // gchasing

    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), thrust));
    while (timed_thrust_.size() > 100)
    {
        timed_thrust_.pop();
    }
}
