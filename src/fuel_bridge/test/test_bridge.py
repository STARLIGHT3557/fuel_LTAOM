#!/usr/bin/env python3
"""Exercise mission admission, battery return, and bridge batch isolation."""
import copy
import math
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs import point_cloud2
from sensor_msgs.msg import BatteryState, PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from uav_controller.msg import FlightStatus, TrajectorySetpoint
from uav_mission_msgs.msg import ControllerAck, MissionStatus, TaskContext
from visualization_msgs.msg import Marker


class BridgeTest(unittest.TestCase):
    def wait_for(self, predicate, timeout=3.0, message='condition not reached'):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        self.fail(message)

    def test_contract(self):
        clouds, hits, poses, commands, triggers, bounds, statuses = ([] for _ in range(7))
        subscribers = [
            rospy.Subscriber('/fuel/cloud_world', PointCloud2, clouds.append),
            rospy.Subscriber('/fuel/cloud_hits_world', PointCloud2, hits.append),
            rospy.Subscriber('/fuel/sensor_pose', PoseStamped, poses.append),
            rospy.Subscriber('/fuel/command', TrajectorySetpoint, commands.append),
            rospy.Subscriber('/fuel/trigger', Path, triggers.append),
            rospy.Subscriber('/fuel/exploration_bounds', Marker, bounds.append),
            rospy.Subscriber('/fuel/mission_status', MissionStatus, statuses.append),
        ]
        cloud_pub = rospy.Publisher('/fuel/cloud', PointCloud2, queue_size=2)
        odom_pub = rospy.Publisher('/fuel/odom', Odometry, queue_size=2)
        state_pub = rospy.Publisher('/fuel/control_state', FlightStatus, queue_size=2)
        battery_pub = rospy.Publisher('/fuel/battery', BatteryState, queue_size=2)
        cmd_pub = rospy.Publisher('/fuel/position_cmd', PositionCommand, queue_size=2)
        ack_pub = rospy.Publisher('/fuel/controller_ack', ControllerAck, queue_size=2)
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        for service in ('/fuel/start', '/fuel/stop', '/fuel/return_home'):
            rospy.wait_for_service(service, timeout=10)
        start = rospy.ServiceProxy('/fuel/start', Trigger)
        stop = rospy.ServiceProxy('/fuel/stop', Trigger)
        return_home = rospy.ServiceProxy('/fuel/return_home', Trigger)
        self.assertFalse(start().success)

        def feed(ready=True, frame='uav1/lidar_link', x=2.0, y=3.0, z=1.5,
                 vx=0.0, battery=0.31):
            stamp = rospy.Time.now()
            odom = Odometry(header=Header(stamp=stamp, frame_id='world'))
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = z
            odom.twist.twist.linear.x = vx
            odom.pose.pose.orientation.z = math.sqrt(0.5)
            odom.pose.pose.orientation.w = math.sqrt(0.5)
            state = FlightStatus(feedback_ready=ready,
                                 output_policy='CLOSED_LOOP' if ready else 'NO_OUTPUT',
                                 phase='HOLD' if ready else 'WAIT')
            charge = BatteryState(header=Header(stamp=stamp), percentage=battery)
            cloud = point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id=frame),
                [(1, 0, 0), (40, 0, 0), (0, 0, 0), (float('nan'), 1, 1)])
            odom_pub.publish(odom)
            state_pub.publish(state)
            battery_pub.publish(charge)
            cloud_pub.publish(cloud)
            time.sleep(0.03)

        for _ in range(5):
            feed(frame='unrelated_sensor_frame')
        self.assertFalse(clouds, 'Unexpected sensor frames must be rejected')
        for _ in range(15):
            feed()
        self.assertTrue(clouds and poses and bounds)
        self.assertEqual(len(bounds[-1].points), 24)
        points = list(point_cloud2.read_points(clouds[-1], field_names=('x', 'y', 'z')))
        self.assertEqual(len(points), 2, 'Keep max-range rays for mapping')
        self.assertEqual(hits[-1].width, 1, 'Do not visualize no-hit endpoints as obstacles')
        for actual, expected in zip(points[0], (2, 4.0865176032, 1.4340708282)):
            self.assertAlmostEqual(actual, expected, places=4)
        self.assertEqual(clouds[-1].header.stamp, poses[-1].header.stamp)

        cmd = PositionCommand()
        cmd.header.frame_id = 'world'
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        cmd.trajectory_id = 1
        cmd.position.x, cmd.position.y, cmd.position.z = 2, 3, 1.5
        cmd.velocity.x, cmd.acceleration.y, cmd.yaw = 0.2, 0.1, 0.7
        cmd.header.stamp = rospy.Time.now()
        cmd_pub.publish(cmd)
        time.sleep(0.05)
        self.assertFalse(commands, 'Commands must be gated before a task is authorized')

        self.assertTrue(start().success)
        self.wait_for(lambda: statuses and statuses[-1].state == MissionStatus.EXPLORING,
                      message='manager did not enter EXPLORING')
        explore = copy.deepcopy(statuses[-1].context)
        self.assertEqual(explore.task_type, TaskContext.EXPLORE)
        self.assertFalse(start().success, 'Duplicate start must not replace an active task')

        cmd.context = explore
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and not any(not item.finished for item in commands):
            feed()
            cmd.header.stamp = rospy.Time.now()
            cmd_pub.publish(cmd)
            time.sleep(0.02)
        self.wait_for(lambda: any(not item.finished for item in commands),
                      message='authorized PositionCommand did not reach controller')
        moves = [item for item in commands if not item.finished]
        self.assertEqual(moves[-1].context, explore)
        self.assertEqual(moves[-1].trajectory_id, 1)
        self.assertAlmostEqual(moves[-1].velocity.x, 0.2, places=5)
        self.assertAlmostEqual(moves[-1].acceleration.y, 0.1, places=5)

        wrong = copy.deepcopy(explore)
        wrong.source_id = 'untrusted_planner'
        cmd.context = wrong
        cmd.trajectory_id = 2
        cmd.header.stamp = rospy.Time.now()
        cmd_pub.publish(cmd)
        time.sleep(0.08)
        self.assertFalse(any(item.context.source_id == 'untrusted_planner' for item in commands),
                         'Wrong-source command crossed the bridge')

        self.assertTrue(return_home().success)
        self.wait_for(lambda: statuses[-1].state == MissionStatus.PREPARE_RETURN,
                      message='manual return did not enter PREPARE_RETURN')
        returning = copy.deepcopy(statuses[-1].context)
        self.assertEqual(returning.task_type, TaskContext.RETURN_HOME)
        self.assertGreater(returning.mission_id, explore.mission_id)
        self.assertTrue(return_home().success)
        time.sleep(0.05)
        self.assertEqual(statuses[-1].context.mission_id, returning.mission_id)

        old_count = len([item for item in commands if not item.finished])
        cmd.context = explore
        cmd.trajectory_id = 99
        cmd.header.stamp = rospy.Time.now()
        cmd_pub.publish(cmd)
        time.sleep(0.08)
        self.assertEqual(old_count, len([item for item in commands if not item.finished]),
                         'Old exploration command entered the return task')

        cmd.context = returning
        cmd.trajectory_id = 1
        for _ in range(3):
            feed()
            cmd.header.stamp = rospy.Time.now()
            cmd_pub.publish(cmd)
        self.wait_for(lambda: any(item.context == returning and not item.finished for item in commands),
                      message='return command did not cross the bridge')
        deadline = time.monotonic() + 5.0
        while ack_pub.get_num_connections() == 0 and time.monotonic() < deadline:
            feed()
        self.assertGreater(ack_pub.get_num_connections(), 0)
        ack = ControllerAck(context=returning, trajectory_id=1, accepted=True)
        for _ in range(3):
            ack.header.stamp = rospy.Time.now()
            ack_pub.publish(ack)
            feed()
        self.wait_for(lambda: statuses[-1].state == MissionStatus.RETURNING,
                      message='controller acknowledgement did not enter RETURNING')
        for _ in range(12):
            feed(x=0.0, y=0.0, z=1.0)
        self.wait_for(lambda: statuses[-1].state == MissionStatus.HOME_HOLD,
                      message='stable physical home did not produce HOME_HOLD')
        self.assertEqual(statuses[-1].return_reason, MissionStatus.MANUAL_RETURN)
        self.wait_for(lambda: commands and commands[-1].finished,
                      message='HOME_HOLD did not issue a finished home setpoint')
        self.assertAlmostEqual(commands[-1].position.z, 1.0)
        self.assertTrue(statuses[-1].mission_done)

        self.assertTrue(stop().success)
        for _ in range(5):
            feed(battery=0.30)
        self.assertTrue(start().success, 'Exactly 30% must still permit exploration')
        self.wait_for(lambda: statuses[-1].state == MissionStatus.EXPLORING)
        low_battery_explore = statuses[-1].context.mission_id
        for _ in range(3):
            feed(battery=0.29)
        self.wait_for(lambda: statuses[-1].state == MissionStatus.PREPARE_RETURN,
                      message='31% to 29% did not trigger return')
        low_return = copy.deepcopy(statuses[-1].context)
        self.assertEqual(statuses[-1].return_reason, MissionStatus.LOW_BATTERY)
        self.assertGreater(low_return.mission_id, low_battery_explore)
        for _ in range(4):
            feed(battery=0.31)
        self.assertEqual(statuses[-1].context, low_return,
                         'Battery recovery must not cancel or recreate return')
        self.assertIn(statuses[-1].state, (MissionStatus.PREPARE_RETURN, MissionStatus.RETURNING))

        self.assertTrue(stop().success)
        for _ in range(5):
            feed(ready=False)
        self.assertFalse(start().success, 'Invalid controller state must reject exploration')
        goal = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='world'))
        goal.pose.orientation.w = 1.0
        previous_id = statuses[-1].context.mission_id
        goal_pub.publish(goal)
        time.sleep(0.1)
        self.assertEqual(statuses[-1].context.mission_id, previous_id,
                         'RViz start must obey the same readiness checks')


if __name__ == '__main__':
    rospy.init_node('test_fuel_bridge')
    rostest.rosrun('prometheus_fuel', 'fuel_bridge_contract', BridgeTest)
