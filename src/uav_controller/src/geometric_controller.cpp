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

// Recursive least squares with vanishing memory.  est_a_z is the measured
// world-up specific force; model: est_a_z = thr2acc_ * thr, where thr is the
// thrust actually applied by the FCU (0..1), not the unsaturated request.
bool GeometricController::estimateThrustModel(double est_a_z, double now_sec) {
  if (!std::isfinite(est_a_z) || !std::isfinite(now_sec)) return false;
  const double nominal = params_.gravity / params_.hover_percentage;
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
    // Only clips the regressor: a request outside (0, 1] cannot have been
    // applied by the FCU, and feeding it back makes the estimate diverge.
    if (!std::isfinite(thr) || thr <= 0.0 || thr > 1.0) return false;

    double gamma = 1.0 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    double next = thr2acc_ + K * (est_a_z - thr * thr2acc_);
    if (!std::isfinite(next)) return false;
    // Bound the estimate to a physically plausible thrust-to-acceleration
    // ratio so a single bad sample can never latch a runaway thrust command.
    thr2acc_ = std::max(0.3 * nominal, std::min(3.0 * nominal, next));
    P_ = std::max(0.0, (1.0 - K * thr) * P_ / rho2_);
    if (!std::isfinite(P_)) P_ = 1e6;
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

  if (estimate_enabled_) estimateThrustModel(st.az_specific_up, now_sec);

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

  // Feed the estimator with the thrust the FCU actually receives. Storing the
  // unsaturated request here made the regressor diverge once it left (0, 1].
  if (std::isfinite(cmd.thrust))
    timed_thrust_.push(std::make_pair(now_sec, cmd.thrust));
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
