// -----------------------------------------------------------------------------
//  GeometricController implementation.
//  Ported (math unchanged) from Prometheus uav_controller.cpp:
//    - send_att_thrust_setpoint()
//    - estimateThrustModel()
//    - resetThrustMapping()
//    - quat2RotMatrix() / rot2Quaternion()
//  ROS types replaced by plain Eigen / std; ros::Time::now() -> now_sec [s].
// -----------------------------------------------------------------------------

#include "uav_controller/geometric_controller.hpp"

#include <algorithm>
#include <cmath>

namespace uav_controller {

GeometricController::GeometricController(const ControllerParams &params)
    : params_(params) {
  thr2acc_ = params_.gravity / params_.hover_percentage;
}

void GeometricController::resetThrustMapping() {
  thr2acc_ = params_.gravity / params_.hover_percentage;
  P_ = 1e6;
  estimate_enabled_ = false;
  std::queue<std::pair<double, double>> empty;
  std::swap(timed_thrust_, empty);
}

// 悬停油门在线估计（带遗忘因子的递推最小二乘，与 px4ctrl 相同）：
// 模型为 机体 z 比力 = thr2acc_ × 油门，悬停油门 = g / thr2acc_。
// 用 35~45 ms 前发出的油门对应当前 IMU 读数，补偿飞控和电机的响应延迟。
// Recursive least squares with vanishing memory.  est_a_z is the measured
// body-z specific force; model: est_a_z = thr2acc_ * thr.
bool GeometricController::estimateThrustModel(double est_a_z, double now_sec) {
  while (timed_thrust_.size() >= 1) {
    // Choose data 35~45 ms old.
    std::pair<double, double> t_t = timed_thrust_.front();
    double time_passed = now_sec - t_t.first;
    if (time_passed > 0.045) {  // 45 ms
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) {  // 35 ms
      return false;
    }

    double thr = t_t.second;
    timed_thrust_.pop();

    double gamma = 1.0 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a_z - thr * thr2acc_);
    P_ = (1.0 - K * thr) * P_ / rho2_;

    // 合理性限幅（同 px4ctrl）：估计的悬停油门限制在 [0.1, 0.8]，
    // 防止振动或触地冲击导致油门饱和或过低。
    // Sanity bounds as in px4ctrl: estimated hover throttle within [0.1, 0.8].
    const double hover_percentage = params_.gravity / thr2acc_;
    if (hover_percentage > 0.8 || hover_percentage < 0.1) {
      thr2acc_ = hover_percentage > 0.8 ? params_.gravity / 0.8 : thr2acc_;
      thr2acc_ = hover_percentage < 0.1 ? params_.gravity / 0.1 : thr2acc_;
    }
    return true;
  }
  return false;
}

AttitudeThrustCommand GeometricController::computeAttitudeThrust(
    const ControllerSetpoint &sp, const ControllerState &st, double now_sec) {
  const double g = params_.gravity;

  Eigen::Vector3d pos_error = sp.pos - st.pos;
  Eigen::Vector3d vel_error = sp.vel - st.vel;

  Eigen::Vector3d a_fb =
      params_.k_pos.asDiagonal() * pos_error + params_.k_vel.asDiagonal() * vel_error;

  if (a_fb.norm() > params_.max_a_fb)
    a_fb = (params_.max_a_fb / a_fb.norm()) * a_fb;

  const Eigen::Vector3d a_des = a_fb + g * Eigen::Vector3d(0, 0, 1) + sp.acc;

  Eigen::Vector4f quat_current;
  quat_current(0) = static_cast<float>(st.quat.w());
  quat_current(1) = static_cast<float>(st.quat.x());
  quat_current(2) = static_cast<float>(st.quat.y());
  quat_current(3) = static_cast<float>(st.quat.z());

  Eigen::Matrix3d rotation_current = quat2RotMatrix(quat_current);

  if (estimate_enabled_) estimateThrustModel(st.az_body, now_sec);

  double thrust = a_des.dot(rotation_current * Eigen::Vector3d(0, 0, 1)) / thr2acc_;

  float psi = static_cast<float>(sp.yaw);
  Eigen::Vector3d proj_xb_des(cosf(psi), sinf(psi), 0);
  Eigen::Vector3d zb_des = a_des / a_des.norm();
  Eigen::Vector3d yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des).norm());
  Eigen::Vector3d xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();
  Eigen::Matrix3d R_des;
  R_des << xb_des(0), yb_des(0), zb_des(0),
           xb_des(1), yb_des(1), zb_des(1),
           xb_des(2), yb_des(2), zb_des(2);

  Eigen::Vector4f quat_des = rot2Quaternion(R_des);

  AttitudeThrustCommand cmd;
  cmd.q = Eigen::Quaterniond(quat_des(0), quat_des(1), quat_des(2), quat_des(3));
  cmd.thrust = std::max(0.0, std::min(1.0, thrust + 0.01));
  cmd.thrust_raw = thrust;  // modeled thrust before output clamp/bias

  // Feed the thrust estimator (keep last ~100 samples), as in the original.
  timed_thrust_.push(std::make_pair(now_sec, thrust));
  while (timed_thrust_.size() > 100) timed_thrust_.pop();

  return cmd;
}

Eigen::Matrix3d GeometricController::quat2RotMatrix(const Eigen::Vector4f &q) {
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

Eigen::Vector4f GeometricController::rot2Quaternion(const Eigen::Matrix3d &R) {
  Eigen::Vector4f quat;
  double tr = R.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0;  // S = 4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S = 4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S = 4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S = 4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}

}  // namespace uav_controller
