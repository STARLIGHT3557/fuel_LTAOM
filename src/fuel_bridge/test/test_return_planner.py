#!/usr/bin/env python3
"""Real FUEL: blocked home must keep retrying, then recover when map clears."""
import copy
import math
import time
import unittest

import rospy
import rostest
from nav_msgs.msg import Odometry
from bspline.msg import Bspline
from sensor_msgs import point_cloud2
from sensor_msgs.msg import BatteryState, PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from uav_controller.msg import FlightStatus, TrajectorySetpoint
from uav_mission_msgs.msg import MissionStatus, PlannerStatus


class PersistentReturnPlannerTest(unittest.TestCase):
    def wait_for(self, predicate, message, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.05)
        self.fail(message)

    def test_map_recovery(self):
        statuses, planning, commands, maps, splines = [], [], [], [], []
        subscribers = [
            rospy.Subscriber("/planning/bspline", Bspline, splines.append),
            rospy.Subscriber('/fuel/mission_status', MissionStatus, statuses.append),
            rospy.Subscriber('/fuel/planner_status', PlannerStatus, planning.append),
            rospy.Subscriber('/uav_controller/trajectory_setpoint', TrajectorySetpoint, commands.append),
            rospy.Subscriber('/sdf_map/occupancy_all', PointCloud2, maps.append),
        ]
        odom_pub = rospy.Publisher('/fuel_test/odom', Odometry, queue_size=2)
        cloud_pub = rospy.Publisher('/fuel_test/cloud', PointCloud2, queue_size=2)
        state_pub = rospy.Publisher('/uav_controller/fsm_status', FlightStatus, queue_size=2)
        battery_pub = rospy.Publisher('/fuel_test/battery', BatteryState, queue_size=2)
        blocked = [True]
        began = time.monotonic()
        pitch_c, pitch_s = math.cos(0.35), math.sin(0.35)
        directions = []
        for az in range(0, 360, 2):
            for el in range(-7, 54, 2):
                azimuth, elevation = math.radians(az), math.radians(el)
                x = math.cos(elevation) * math.cos(azimuth)
                y = math.cos(elevation) * math.sin(azimuth)
                z = math.sin(elevation)
                directions.append((x, y, z, pitch_c*x + pitch_s*z, -pitch_s*x + pitch_c*z))

        def feed(_):
            stamp = rospy.Time.now()
            height = min(1.5, 0.2 + 0.5 * (time.monotonic() - began))
            odom = Odometry(header=Header(stamp=stamp, frame_id='world'))
            odom.pose.pose.position.z = height
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            # Report flying: this odometry never follows the trajectory, and the bridge refuses
            # references that move away from a hovering aircraft.
            state_pub.publish(FlightStatus(feedback_ready=True, output_policy='CLOSED_LOOP', phase='TRAJECTORY'))
            battery_pub.publish(BatteryState(header=Header(stamp=stamp), percentage=0.31))
            points = []
            for x, y, z, dx, dz in directions:
                distance = 12.0
                if blocked[0] and dx > 0.01:
                    wall = (3.0 - 0.1471448904) / dx
                    if abs(wall*y) < 1.5 and 0.0 < height + 0.2769686356 + wall*dz < 3.0:
                        distance = min(distance, wall)
                # Model a floor return. Rays ending below the map's ground
                # cutoff are discarded before raycasting, so an infinite
                # downward no-hit ray cannot clear the former wall.
                if dz < -1e-6:
                    floor_distance = (0.25 - height - 0.2769686356) / dz
                    if floor_distance > 0:
                        distance = min(distance, floor_distance)
                points.append((distance*x, distance*y, distance*z))
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id='lidar_link'), points))

        timer = rospy.Timer(rospy.Duration(0.1), feed)
        try:
            rospy.wait_for_service('/fuel/return_home', timeout=15)
            self.wait_for(lambda: time.monotonic()-began > 5 and statuses and statuses[-1].battery_valid
                          and any(m.width for m in maps), 'synthetic wall/map was not ready')
            self.assertTrue(rospy.ServiceProxy('/fuel/return_home', Trigger)().success)
            self.wait_for(lambda: any(s.state == PlannerStatus.OBSERVING_HOME for s in planning),
                          'no observation heartbeat at blocked home')
            context = copy.deepcopy(statuses[-1].context)
            self.assertEqual(statuses[-1].return_reason, MissionStatus.MANUAL_RETURN)
            blocked[0] = False
            def home_splines():
                return [b for b in splines if b.context == context and
                        all(abs(p.x - 3.0) < 1e-6 and abs(p.y) < 1e-6 and abs(p.z - 1.0) < 1e-6
                            for p in b.pos_pts[-3:])]
            self.wait_for(home_splines, 'return to the original home was not planned', 30)
            terminal = home_splines()[-1]
            self.assertGreaterEqual(terminal.knots[-terminal.order-1] - terminal.knots[terminal.order], 2.0)
            # Repeated final control points enforce exact home and zero v/a.
            for point in terminal.pos_pts[-3:]:
                self.assertAlmostEqual(point.x, 3.0, places=6)
                self.assertAlmostEqual(point.y, 0.0, places=6)
                self.assertAlmostEqual(point.z, 1.0, places=6)
            for heading in terminal.yaw_pts[-3:]:
                self.assertAlmostEqual(heading, 0.0, places=6)
            self.assertTrue(any(s.context == context and s.state == PlannerStatus.TRAJECTORY_READY
                                for s in planning))

            # Home blocked after the terminal segment: the arrival is withdrawn on the same
            # trajectory, replanning waits out map noise, and home is retried until it clears.
            self.wait_for(lambda: planning[-1].state == PlannerStatus.GOAL_REACHED,
                          'terminal trajectory did not finish', 30)
            finished = planning[-1].trajectory_id
            checkpoint = len(planning)
            blocked[0] = True
            self.wait_for(lambda: any(s.state == PlannerStatus.OBSERVING_HOME
                                      for s in planning[checkpoint:]), 'blocked home was not observed', 30)
            withdrawn = next(s for s in planning[checkpoint:] if s.state == PlannerStatus.EXECUTING
                             and 'waiting for home' in s.detail)
            replacement = next(s for s in planning[checkpoint:]
                               if s.state == PlannerStatus.TRAJECTORY_READY)
            self.assertEqual(withdrawn.trajectory_id, finished)
            self.assertGreaterEqual((replacement.header.stamp - withdrawn.header.stamp).to_sec(), 0.45,
                                    'map noise would replan the terminal segment')
            blocked[0] = False
            self.wait_for(lambda: any(s.state == PlannerStatus.TRAJECTORY_READY
                                      for s in planning[checkpoint:]), 'home was not retried after it cleared')
            self.wait_for(lambda: any(s.state == PlannerStatus.GOAL_REACHED and s.trajectory_id > finished
                                      for s in planning[checkpoint:]), 'new terminal trajectory did not finish', 30)
            self.assertTrue(rospy.ServiceProxy('/fuel/stop', Trigger)().success)
        finally:
            timer.shutdown()


if __name__ == '__main__':
    rospy.init_node('test_return_planner')
    rostest.rosrun('fuel_bridge', 'persistent_return_planner', PersistentReturnPlannerTest)
