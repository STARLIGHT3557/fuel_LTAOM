#!/usr/bin/env python3
"""Planner smoke test with synthetic ray endpoints; not a flight dynamics test."""
import math
import time
import unittest

import rospy
import rostest
from bspline.msg import Bspline
from nav_msgs.msg import Odometry
from uav_controller.msg import FlightStatus, TrajectorySetpoint
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker


class PlannerTest(unittest.TestCase):
    def test_planner(self):
        self.moves, self.splines, self.frontiers, self.maps = [], [], [], []
        subscribers = [
            rospy.Subscriber('/uav_controller/trajectory_setpoint', TrajectorySetpoint, self.moves.append),
            rospy.Subscriber('/planning/bspline', Bspline, self.splines.append),
            rospy.Subscriber('/planning_vis/frontier', Marker, self.frontiers.append),
            rospy.Subscriber('/sdf_map/occupancy_all', PointCloud2, self.maps.append),
        ]
        odom_pub = rospy.Publisher('/fuel_test/odom', Odometry, queue_size=2)
        cloud_pub = rospy.Publisher('/fuel_test/cloud', PointCloud2, queue_size=2)
        state_pub = rospy.Publisher('/uav_controller/fsm_status', FlightStatus, queue_size=2)
        # Long returns let FUEL raycast free space to its 10 m range limit.
        # A small wall at x=4 provides occupied voxels without closing the room.
        points = []
        for degrees in range(0, 360, 2):
            azimuth = math.radians(degrees)
            for elevation in range(-7, 54, 2):
                el = math.radians(elevation)
                x, y, z = math.cos(el) * math.cos(azimuth), math.cos(el) * math.sin(azimuth), math.sin(el)
                distance = 12.0
                if x > 0.01:
                    wall_distance = 4.0 / x
                    if abs(wall_distance * y) < 1 and 0 < 1.78 + wall_distance * z < 3:
                        distance = min(distance, wall_distance)
                points.append((distance * x, distance * y, distance * z))

        scan_start = time.monotonic()

        def feed(_):
            stamp = rospy.Time.now()
            odom = Odometry(header=Header(stamp=stamp, frame_id='world'))
            # Scan throughout takeoff, as in the real launch sequence. MID360's
            # shallow downward FOV cannot observe the body from a stationary
            # sensor mounted 0.28 m above it without this initial map history.
            odom.pose.pose.position.z = min(1.5, 0.2 + 0.4 * (time.monotonic() - scan_start))
            odom.pose.pose.orientation.w = 1
            odom_pub.publish(odom)
            state_pub.publish(FlightStatus(feedback_ready=True, output_policy="CLOSED_LOOP",
                                           phase="HOLD", mission_id=42))
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id='uav1/lidar_link'), points))

        timer = rospy.Timer(rospy.Duration(0.1), feed)
        try:
            rospy.wait_for_service('/fuel/start', timeout=20)
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                if (time.monotonic() - scan_start > 5 and
                        any(m.width for m in self.maps) and any(m.points for m in self.frontiers)):
                    break
                time.sleep(0.1)
            self.assertTrue(any(m.width for m in self.maps), 'No occupied map produced')
            self.assertTrue(any(m.points for m in self.frontiers), 'No frontiers produced')
            response = rospy.ServiceProxy('/fuel/start', Trigger)()
            self.assertTrue(response.success, response.message)
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline and not any(not c.finished for c in self.moves):
                time.sleep(0.1)
            self.assertTrue(self.splines, 'FUEL produced no B-spline')
            self.assertTrue(any(not c.finished for c in self.moves), 'No controller command')
            for marker in self.frontiers:
                if marker.action == Marker.ADD:
                    self.assertTrue(marker.points, 'RViz rejects an empty frontier ADD marker')
            for spline in self.splines:
                for point in spline.pos_pts:
                    self.assertTrue(-1.0 < point.x < 22.0)
                    self.assertTrue(-11.0 < point.y < 10.0)
                    self.assertTrue(0.5 < point.z < 2.5)
            self.assertTrue(rospy.ServiceProxy('/fuel/stop', Trigger)().success)
        finally:
            timer.shutdown()


if __name__ == '__main__':
    rospy.init_node('test_fuel_planner')
    rostest.rosrun('prometheus_fuel', 'fuel_planner_smoke', PlannerTest)
