#!/usr/bin/env python3
"""Forward each valid EKF pose to MAVROS without changing coordinate frames."""

import copy
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class EkfToPx4:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_publish_stamp = None
        self.pub = rospy.Publisher("~pose", PoseStamped, queue_size=100)
        self.odom_sub = rospy.Subscriber(
            "~odom", Odometry, self.odom_callback,
            queue_size=100, buff_size=262144, tcp_nodelay=True)
        rospy.loginfo("EKF pose forwarding enabled for every valid input frame; "
                      "pose axes and origin must already be configured.")

    @staticmethod
    def pose_is_valid(pose):
        p, q = pose.position, pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = math.hypot(q.x, q.y, q.z, q.w)
        return (all(math.isfinite(v) for v in values)
                and math.isfinite(norm) and norm > 1e-6)

    def odom_callback(self, msg):
        if (not msg.header.frame_id or not self.pose_is_valid(msg.pose.pose)
                or msg.header.stamp.to_sec() <= 0.0):
            rospy.logwarn_throttle(2.0, "Dropping EKF pose: invalid pose/frame/stamp.")
            return

        with self.lock:
            if self.last_publish_stamp is not None:
                delta = (msg.header.stamp - self.last_publish_stamp).to_sec()
                if delta <= 0.0:
                    rospy.logwarn_throttle(2.0, "Dropping non-increasing EKF "
                                           "stamp; restart after clock reset.")
                    return

            out = PoseStamped()
            out.header = copy.deepcopy(msg.header)
            out.pose = copy.deepcopy(msg.pose.pose)
            q = out.pose.orientation
            norm = math.hypot(q.x, q.y, q.z, q.w)
            q.x, q.y, q.z, q.w = (q.x / norm, q.y / norm,
                                  q.z / norm, q.w / norm)
            self.pub.publish(out)
            self.last_publish_stamp = msg.header.stamp


if __name__ == "__main__":
    rospy.init_node("ekf_to_px4")
    node = EkfToPx4()
    rospy.spin()
