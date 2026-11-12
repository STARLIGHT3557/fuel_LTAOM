#!/usr/bin/env python3
import math
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class RealflightBridgeTest(unittest.TestCase):
    def test_deskewed_imu_cloud_to_px4_enu(self):
        clouds, poses = [], []
        subscribers = [
            rospy.Subscriber('/realflight_test/cloud_world', PointCloud2, clouds.append),
            rospy.Subscriber('/realflight_test/sensor_pose', PoseStamped, poses.append)]
        cloud_pub = rospy.Publisher('/realflight_test/cloud', PointCloud2, queue_size=2)
        odom_pub = rospy.Publisher('/realflight_test/odom', Odometry, queue_size=2)
        deadline = time.monotonic() + 5
        while (cloud_pub.get_num_connections() == 0 or odom_pub.get_num_connections() == 0) and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(cloud_pub.get_num_connections() and odom_pub.get_num_connections())

        def feed(frame='imu', age=0., skew=0., odom_frame='map', valid=True):
            stamp = rospy.Time.now() - rospy.Duration(age)
            odom = Odometry(header=Header(stamp=stamp + rospy.Duration(skew), frame_id=odom_frame))
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = 10, 20, 3
            odom.pose.pose.orientation.z = math.sqrt(0.5) if valid else 0.
            odom.pose.pose.orientation.w = math.sqrt(0.5) if valid else 0.
            odom_pub.publish(odom)
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id=frame),
                              [(2, 0, 0), (0, 0, 0.1), (float('nan'), 0, 0)]))
            time.sleep(0.04)

        for _ in range(4):
            feed(frame='world')
            feed(age=1.)
            feed(odom_frame='other_map')
            feed(valid=False)
        self.assertFalse(clouds, 'Invalid frames, stale data and invalid orientation must be rejected')
        for _ in range(12):
            feed()
        self.assertTrue(clouds and poses)
        self.assertEqual(clouds[-1].header.frame_id, 'map')
        points = list(point_cloud2.read_points(clouds[-1], field_names=('x', 'y', 'z')))
        self.assertEqual(len(points), 1, 'Filter near returns relative to actual LiDAR origin')
        for actual, expected in zip(points[0], (10.1, 20.3, 1.2)):
            self.assertAlmostEqual(actual, expected, places=4)
        origin = poses[-1].pose.position
        for actual, expected in zip((origin.x, origin.y, origin.z), (10.1, 20.4, 3.2)):
            self.assertAlmostEqual(actual, expected, places=4)
        self.assertEqual(poses[-1].header, clouds[-1].header)
        q = poses[-1].pose.orientation
        for actual in (q.x, q.y, q.z, q.w):
            self.assertAlmostEqual(abs(actual), 0.5, places=4)


if __name__ == '__main__':
    rospy.init_node('test_realflight_bridge')
    rostest.rosrun('fuel_bridge', 'realflight_cloud_transform', RealflightBridgeTest)
