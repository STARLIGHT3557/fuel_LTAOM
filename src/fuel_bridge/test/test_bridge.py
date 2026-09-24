#!/usr/bin/env python3
"""Exercise the actual ROS bridge with synthetic sensor and planner messages."""
import math
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from uav_controller.msg import FlightStatus, TrajectorySetpoint
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker


class BridgeTest(unittest.TestCase):
    def test_contract(self):
        clouds, hits, poses, commands, triggers, bounds = [], [], [], [], [], []
        subscribers = [
            rospy.Subscriber('/fuel/cloud_world', PointCloud2, clouds.append),
            rospy.Subscriber('/fuel/cloud_hits_world', PointCloud2, hits.append),
            rospy.Subscriber('/fuel/sensor_pose', PoseStamped, poses.append),
            rospy.Subscriber('/fuel/command', TrajectorySetpoint, commands.append),
            rospy.Subscriber('/fuel/trigger', Path, triggers.append),
            rospy.Subscriber('/fuel/exploration_bounds', Marker, bounds.append),
        ]
        cloud_pub = rospy.Publisher('/fuel/cloud', PointCloud2, queue_size=2)
        odom_pub = rospy.Publisher('/fuel/odom', Odometry, queue_size=2)
        state_pub = rospy.Publisher('/fuel/control_state', FlightStatus, queue_size=2)
        cmd_pub = rospy.Publisher('/fuel/position_cmd', PositionCommand, queue_size=2)
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.wait_for_service('/fuel/start', timeout=10)
        start = rospy.ServiceProxy('/fuel/start', Trigger)
        stop = rospy.ServiceProxy('/fuel/stop', Trigger)
        self.assertFalse(start().success)
        time.sleep(0.3)

        def feed(ready=True, frame='uav1/lidar_link', x=2, z=1.5, vx=0.0):
            stamp = rospy.Time.now()
            odom = Odometry(header=Header(stamp=stamp, frame_id='world'))
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = 3
            odom.pose.pose.position.z = z
            odom.twist.twist.linear.x = vx
            odom.pose.pose.orientation.z = math.sqrt(0.5)
            odom.pose.pose.orientation.w = math.sqrt(0.5)
            state = FlightStatus(feedback_ready=ready, output_policy="CLOSED_LOOP" if ready else "NO_OUTPUT",
                                 phase="HOLD" if ready else "WAIT", mission_id=42)
            cloud = point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id=frame),
                [(1, 0, 0), (40, 0, 0), (0, 0, 0), (float('nan'), 1, 1)])
            odom_pub.publish(odom)
            state_pub.publish(state)
            cloud_pub.publish(cloud)
            time.sleep(0.03)

        for _ in range(5):
            feed(frame='unrelated_sensor_frame')
        self.assertFalse(clouds, 'Unexpected sensor frames must still be rejected')
        for _ in range(15):
            feed()
        self.assertTrue(clouds and poses)
        self.assertTrue(bounds)
        self.assertEqual(len(bounds[-1].points), 24)
        self.assertEqual(bounds[-1].type, Marker.LINE_LIST)
        self.assertEqual(min(p.x for p in bounds[-1].points), -1)
        self.assertEqual(max(p.x for p in bounds[-1].points), 22)
        points = list(point_cloud2.read_points(clouds[-1], field_names=('x', 'y', 'z')))
        self.assertEqual(len(points), 2, 'Keep max-range rays for mapping')
        self.assertTrue(hits)
        self.assertEqual(hits[-1].width, 1, 'Do not visualize no-hit endpoints as obstacles')
        # 0.35 rad mounting pitch, 0.05 m ray offset, and a 90 degree body yaw.
        for actual, expected in zip(points[0], (2, 4.0865176032, 1.4340708282)):
            self.assertAlmostEqual(actual, expected, places=4)
        self.assertEqual(clouds[-1].header.stamp, poses[-1].header.stamp)
        self.assertAlmostEqual(poses[-1].pose.position.y, 3.1471448904, places=4)
        cmd = PositionCommand()
        cmd.header.frame_id = 'world'
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        cmd.position.x, cmd.position.y, cmd.position.z = 2, 3, 1.5
        cmd.velocity.x, cmd.acceleration.y, cmd.yaw = 0.2, 0.1, 0.7
        cmd.header.stamp = rospy.Time.now()
        cmd_pub.publish(cmd)
        time.sleep(0.05)
        self.assertFalse(commands, 'Commands must be gated before exploration starts')
        # RViz publishes a pose on this topic; no start service call is needed.
        self.assertGreater(goal_pub.get_num_connections(), 0)
        goal = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='world'))
        goal.pose.position.x = 100  # Click location is a trigger, not a destination.
        goal.pose.orientation.w = 1
        # Height and speed no longer gate exploration startup.
        feed(z=0.6, vx=2.0)
        self.assertTrue(start().success)
        self.assertTrue(stop().success)
        time.sleep(0.08)
        commands.clear()
        triggers.clear()
        for _ in range(3):
            feed()
        goal_pub.publish(goal)
        for _ in range(5):
            feed()
            cmd.header.stamp = rospy.Time.now()
            cmd_pub.publish(cmd)
        time.sleep(0.05)
        self.assertTrue(triggers and commands)
        self.assertEqual(len(triggers[-1].poses), 1)
        self.assertAlmostEqual(triggers[-1].poses[0].pose.position.x, 2)
        goal_pub.publish(goal)
        feed()
        self.assertEqual(len(triggers), 1, 'Repeated clicks must not restart active exploration')
        moves = [c for c in commands if not c.finished]
        self.assertGreaterEqual(len(moves), 2)
        self.assertAlmostEqual(moves[-1].velocity.x, 0.2, places=5)
        self.assertAlmostEqual(moves[-1].acceleration.y, 0.1, places=5)
        self.assertAlmostEqual(moves[-1].yaw, 0.7, places=5)
        self.assertEqual(moves[-1].mission_id, 42)

        # Fresh sensors but a silent trajectory server must request hover.
        for _ in range(25):
            feed()
        self.assertTrue(commands[-1].finished)
        self.assertTrue(start().success)
        self.assertTrue(stop().success)
        before = len([c for c in commands if not c.finished])
        cmd.header.stamp = rospy.Time.now()
        cmd_pub.publish(cmd)
        time.sleep(0.08)
        self.assertEqual(before, len([c for c in commands if not c.finished]))
        for _ in range(5):
            feed()
        self.assertTrue(start().success)
        before_unfinished = len([c for c in commands if not c.finished])
        cmd.position.x = 23.0
        cmd.header.stamp = rospy.Time.now()
        cmd_pub.publish(cmd)
        time.sleep(0.08)
        unfinished = [c for c in commands if not c.finished]
        self.assertGreater(len(unfinished), before_unfinished)
        self.assertEqual(unfinished[-1].position.x, 23.0)
        command_count = len(commands)
        cmd.position.x = float('nan')
        cmd.header.stamp = rospy.Time.now()
        cmd_pub.publish(cmd)
        time.sleep(0.08)
        self.assertEqual(len(commands), command_count, 'Nonfinite command must be ignored without stopping')
        cmd.position.x = 2.0
        self.assertTrue(stop().success)
        for _ in range(5):
            feed(x=23.0)
        response = start()
        self.assertFalse(response.success)
        self.assertIn('outside', response.message)
        for _ in range(5):
            feed(False)
        self.assertFalse(start().success)
        trigger_count = len(triggers)
        goal_pub.publish(goal)
        for _ in range(3):
            feed(False)
        self.assertEqual(len(triggers), trigger_count, 'RViz must respect the same start conditions')


if __name__ == '__main__':
    rospy.init_node('test_fuel_bridge')
    rostest.rosrun('prometheus_fuel', 'fuel_bridge_contract', BridgeTest)
