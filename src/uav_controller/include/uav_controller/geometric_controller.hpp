#ifndef UAV_CONTROLLER_GEOMETRIC_CONTROLLER_HPP
#define UAV_CONTROLLER_GEOMETRIC_CONTROLLER_HPP

// -----------------------------------------------------------------------------
//  GeometricController
//
//  Faithful, ROS-free port of UAV_controller::send_att_thrust_setpoint() from
//  the Prometheus ROS1 project (Modules/uav_control/src/uav_controller.cpp),
//  together with the recursive-least-squares thrust-to-acceleration estimator
//  (estimateThrustModel) and resetThrustMapping().
//
//  All math is in ENU (z-up). The controller takes the current state and a
//  position/velocity/acceleration/yaw setpoint and returns a desired attitude
//  (quaternion, FLU-in-ENU) plus a normalized collective thrust in [0, 1].
// -----------------------------------------------------------------------------

#include <queue>
#include <utility>
#include <Eigen/Dense>

namespace uav_controller {

struct ControllerParams {
  // PD feedback gains (per-axis, ENU), matching config/params.yaml.
  Eigen::Vector3d k_pos{3.0, 3.0, 4.0};
  Eigen::Vector3d k_vel{3.0, 3.0, 3.5};
  double hover_percentage{0.65};  // throttle at hover -> thr2acc init = g/hover
  double max_a_fb{8.0};           // saturation on PD feedback accel [m/s^2]
  double gravity{9.81};
};

struct ControllerState {
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};      // ENU position [m]
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};      // ENU velocity [m/s]
  Eigen::Quaterniond quat{Eigen::Quaterniond::Identity()};  // FLU-in-ENU attitude
  double az_body{9.81};           // body-z specific force [m/s^2] for thrust est
};

struct ControllerSetpoint {
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};   // ENU desired position [m]
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};   // ENU desired velocity [m/s]
  Eigen::Vector3d acc{Eigen::Vector3d::Zero()};   // ENU feed-forward accel [m/s^2]
  double yaw{0.0};                                // ENU desired yaw [rad]
};

struct AttitudeThrustCommand {
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};  // desired FLU-in-ENU
  double thrust{0.0};       // normalized collective thrust [0, 1] (clamped, +0.01)
  double thrust_raw{0.0};   // modeled thrust before clamp/bias; not a measurement
};

class GeometricController {
public:
  explicit GeometricController(const ControllerParams &params);

  // Main control law. now_sec is a monotonic time stamp in seconds (used by the
  // thrust estimator's 35~45 ms delayed correlation, exactly as in the original).
  AttitudeThrustCommand computeAttitudeThrust(const ControllerSetpoint &sp,
                                              const ControllerState &st,
                                              double now_sec);

  // Reset the thrust mapping (call whenever leaving OFFBOARD / disarmed).
  void resetThrustMapping();

  // Enable/disable online thrust-model estimation.
  void setEstimateEnabled(bool enabled) { estimate_enabled_ = enabled; }

  double thr2acc() const { return thr2acc_; }
  const ControllerParams &params() const { return params_; }

private:
  // RLS update of thr2acc_ from the measured body-z specific force; the
  // estimated hover throttle g/thr2acc_ is clamped to [0.1, 0.8].
  bool estimateThrustModel(double est_a_z, double now_sec);

  static Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4f &q);
  static Eigen::Vector4f rot2Quaternion(const Eigen::Matrix3d &R);

  ControllerParams params_;

  // Thrust mapping estimator state (names match the original).
  const double rho2_{0.998};   // do not change
  double thr2acc_;
  double P_{1e6};
  bool estimate_enabled_{false};
  std::queue<std::pair<double, double>> timed_thrust_;  // (time_sec, thrust)
};

}  // namespace uav_controller

#endif  // UAV_CONTROLLER_GEOMETRIC_CONTROLLER_HPP
