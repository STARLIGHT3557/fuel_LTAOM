#include "uav_controller.h"
#include "uav_estimator.h"
int main(int argc, char** argv) {
  ros::init(argc, argv, "uav_control_main");
  ros::NodeHandle nh("~");
  Eigen::Vector3d kp, kv;
  nh.param("kp_x", kp.x(), 8.0); nh.param("kp_y", kp.y(), 8.0); nh.param("kp_z", kp.z(), 12.0);
  nh.param("kv_x", kv.x(), 4.0); nh.param("kv_y", kv.y(), 4.0); nh.param("kv_z", kv.z(), 5.0);
  UAV_estimator estimator(nh);
  UAV_controller controller(nh, kp, kv);
  ros::Rate rate(250);
  while (ros::ok()) { ros::spinOnce(); controller.mainloop(); rate.sleep(); }
}
