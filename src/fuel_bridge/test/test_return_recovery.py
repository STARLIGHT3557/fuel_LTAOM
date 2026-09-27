#!/usr/bin/env python3
"""Real manager/bridge/executor: persistent failures, timeout and brake recovery."""
import copy
import time
import unittest

import rospy
import rostest
from bspline.msg import Bspline
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs import point_cloud2
from sensor_msgs.msg import BatteryState, PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from uav_controller.msg import FlightStatus, TrajectorySetpoint
from uav_mission_msgs.msg import (ControllerAck, MissionCommand, MissionStatus,
                                 PlannerStatus, TrajectoryStatus)


class ReturnRecoveryTest(unittest.TestCase):
    def wait_for(self, predicate, message, timeout=6.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.025)
        self.fail(message)

    def test_retry_and_resume(self):
        statuses, tasks, points, handoffs = [], [], [], []
        subscribers = [
            rospy.Subscriber("/fuel/trigger", Path, lambda _: None),
            rospy.Subscriber("/fuel/cloud_world", PointCloud2, lambda _: None),
            rospy.Subscriber('/fuel/mission_status', MissionStatus, statuses.append),
            rospy.Subscriber('/fuel/mission_command', MissionCommand, tasks.append),
            rospy.Subscriber('/fuel/command', TrajectorySetpoint, points.append),
            rospy.Subscriber('/fuel/trajectory_status', TrajectoryStatus, handoffs.append),
        ]
        odom_pub = rospy.Publisher('/fuel/odom', Odometry, queue_size=5)
        state_pub = rospy.Publisher('/fuel/control_state', FlightStatus, queue_size=5)
        battery_pub = rospy.Publisher('/fuel/battery', BatteryState, queue_size=5)
        cloud_pub = rospy.Publisher('/fuel/cloud', PointCloud2, queue_size=5)
        planner_pub = rospy.Publisher('/fuel/planner_status', PlannerStatus, queue_size=20)
        spline_pub = rospy.Publisher('/planning/bspline', Bspline, queue_size=5)
        ack_pub = rospy.Publisher('/fuel/controller_ack', ControllerAck, queue_size=5)
        velocity = [0.0]

        def feed(_):
            stamp = rospy.Time.now()
            odom = Odometry(header=Header(stamp=stamp, frame_id='world'))
            odom.pose.pose.position = Point(2.0, 0.0, 1.0)
            # At yaw +90 degrees, body -Y is world +X.
            odom.pose.pose.orientation.w = 0.7071067811865476
            odom.pose.pose.orientation.z = 0.7071067811865476
            odom.twist.twist.linear.y = -velocity[0]
            odom_pub.publish(odom)
            state_pub.publish(FlightStatus(header=Header(stamp=stamp), feedback_ready=True,
                                           output_policy='CLOSED_LOOP', phase='HOLD'))
            battery_pub.publish(BatteryState(header=Header(stamp=stamp), percentage=0.31))
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id='uav1/lidar_link'), [(2, 0, 0)]))
            if handoffs and handoffs[-1].state == TrajectoryStatus.PENDING:
                handoff = handoffs[-1]
                ack_pub.publish(ControllerAck(header=Header(stamp=stamp), context=handoff.context,
                                              trajectory_id=handoff.trajectory_id, accepted=True))

        timer = rospy.Timer(rospy.Duration(0.03), feed)
        try:
            rospy.wait_for_service('/fuel/return_home', timeout=10)
            self.wait_for(lambda: planner_pub.get_num_connections() >= 2 and
                          ack_pub.get_num_connections() >= 2 and statuses and statuses[-1].battery_valid,
                          'test publishers were not connected')
            time.sleep(0.2)
            return_home = rospy.ServiceProxy('/fuel/return_home', Trigger)
            self.assertTrue(return_home().success)
            self.wait_for(lambda: statuses[-1].state == MissionStatus.PREPARE_RETURN,
                          'return task was not created')
            context = copy.deepcopy(statuses[-1].context)
            for count in range(1, 8):
                planner_pub.publish(PlannerStatus(header=Header(stamp=rospy.Time.now()),
                    context=context, state=PlannerStatus.FAILED, consecutive_failures=count))
                time.sleep(0.12)
            self.wait_for(lambda: statuses[-1].state == MissionStatus.RETURN_RECOVERY,
                          'planning failures must enter nonterminal recovery')
            self.assertEqual(statuses[-1].context, context)
            self.assertEqual(statuses[-1].return_reason, MissionStatus.MANUAL_RETURN)
            self.assertFalse(any(t.command == MissionCommand.STOP for t in tasks))
            self.assertTrue(return_home().success, 'repeated return is idempotent during recovery')
            self.assertFalse(rospy.ServiceProxy('/fuel/start', Trigger)().success)
            self.wait_for(lambda: any(t.command == MissionCommand.REPLAN for t in tasks),
                          'missing output must request same-batch replan')
            self.assertTrue(all(t.context == context for t in tasks if t.context.mission_id))

            def send_spline(identifier, x):
                spline = Bspline(context=context, order=3, traj_id=identifier,
                                 start_time=rospy.Time.now() + rospy.Duration(0.1), yaw_dt=0.2)
                spline.pos_pts = [Point(x, 0.0, 1.0) for _ in range(7)]
                spline.knots = [(i - 3) * 0.2 for i in range(11)]
                spline.yaw_pts = [0.0] * 7
                spline_pub.publish(spline)

            send_spline(1, 2.0)
            self.wait_for(lambda: any(p.trajectory_id == 1 and not p.finished for p in points),
                          'bridge did not reopen for recovered return output')
            self.wait_for(lambda: statuses[-1].state == MissionStatus.RETURNING,
                          'valid replacement did not resume return')

            velocity[0] = 0.4
            time.sleep(0.1)
            planner_pub.publish(PlannerStatus(header=Header(stamp=rospy.Time.now()),
                context=context, trajectory_id=1, state=PlannerStatus.TRAJECTORY_UNSAFE))
            self.wait_for(lambda: any(t.state == TrajectoryStatus.INVALID for t in handoffs),
                          'unsafe execution did not start braking')
            self.wait_for(lambda: statuses[-1].state == MissionStatus.RETURN_RECOVERY,
                          'unsafe execution must preserve the return task')
            time.sleep(0.5)
            self.assertTrue(any(abs(p.position.x - 2.08) < 1e-4 and
                                abs(p.position.y) < 1e-4 and not p.finished for p in points),
                            'Body-frame velocity was not rotated before world-frame braking')
            velocity[0] = 0.0
            send_spline(2, 2.08)  # v^2/(2a) = 0.08 m at the default 1 m/s^2 brake.
            self.wait_for(lambda: any(p.trajectory_id == 2 and not p.finished for p in points),
                          'executor stayed stuck in braking instead of accepting replacement')
            self.wait_for(lambda: statuses[-1].state == MissionStatus.RETURNING,
                          'brake recovery did not resume return')
            self.assertEqual(statuses[-1].context, context)
            self.assertEqual(statuses[-1].return_reason, MissionStatus.MANUAL_RETURN)

            self.assertTrue(rospy.ServiceProxy('/fuel/stop', Trigger)().success)
            self.wait_for(lambda: statuses[-1].state == MissionStatus.READY, 'explicit stop failed')
            time.sleep(0.2)
            count = len(tasks)
            planner_pub.publish(PlannerStatus(context=context, state=PlannerStatus.OUTPUT_TIMEOUT))
            time.sleep(0.4)
            self.assertFalse(any(t.command == MissionCommand.REPLAN for t in tasks[count:]),
                             'late failure restarted an explicitly stopped task')
        finally:
            timer.shutdown()


if __name__ == '__main__':
    rospy.init_node('test_return_recovery')
    rostest.rosrun('prometheus_fuel', 'return_recovery_contract', ReturnRecoveryTest)
