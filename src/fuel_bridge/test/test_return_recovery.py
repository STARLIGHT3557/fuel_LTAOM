#!/usr/bin/env python3
"""Real manager/bridge/executor: persistent failures, output timeout and resume."""
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
from uav_mission_msgs.msg import (MissionCommand, MissionStatus, PlannerStatus,
                                 TrajectoryStatus)


class ReturnRecoveryTest(unittest.TestCase):
    def wait_for(self, predicate, message, timeout=6.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.025)
        self.fail(message)

    def test_retry_and_resume(self):
        statuses, tasks, points, executions = [], [], [], []
        subscribers = [
            rospy.Subscriber("/fuel/trigger", Path, lambda _: None),
            rospy.Subscriber("/fuel/cloud_world", PointCloud2, lambda _: None),
            rospy.Subscriber('/fuel/mission_status', MissionStatus, statuses.append),
            rospy.Subscriber('/fuel/mission_command', MissionCommand, tasks.append),
            rospy.Subscriber('/fuel/command', TrajectorySetpoint, points.append),
            rospy.Subscriber('/fuel/trajectory_status', TrajectoryStatus, executions.append),
        ]
        odom_pub = rospy.Publisher('/fuel/odom', Odometry, queue_size=5)
        state_pub = rospy.Publisher('/fuel/control_state', FlightStatus, queue_size=5)
        battery_pub = rospy.Publisher('/fuel/battery', BatteryState, queue_size=5)
        cloud_pub = rospy.Publisher('/fuel/cloud', PointCloud2, queue_size=5)
        planner_pub = rospy.Publisher('/fuel/planner_status', PlannerStatus, queue_size=20)
        spline_pub = rospy.Publisher('/planning/bspline', Bspline, queue_size=5)

        def feed(_):
            stamp = rospy.Time.now()
            odom = Odometry(header=Header(stamp=stamp, frame_id='world'))
            odom.pose.pose.position = Point(2.0, 0.0, 1.0)
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            state_pub.publish(FlightStatus(header=Header(stamp=stamp), feedback_ready=True,
                                           output_policy='CLOSED_LOOP', phase='HOLD'))
            battery_pub.publish(BatteryState(header=Header(stamp=stamp), percentage=0.31))
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id='lidar_link'), [(2, 0, 0)]))

        timer = rospy.Timer(rospy.Duration(0.03), feed)
        try:
            rospy.wait_for_service('/fuel/return_home', timeout=10)
            self.wait_for(lambda: planner_pub.get_num_connections() >= 1 and
                          spline_pub.get_num_connections() >= 1 and
                          statuses and statuses[-1].battery_valid,
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

            # FUEL publishes a B-spline only once its start time has passed.
            spline = Bspline(context=context, order=3, traj_id=1,
                             start_time=rospy.Time.now(), yaw_dt=0.2)
            spline.pos_pts = [Point(2.0, 0.0, 1.0) for _ in range(7)]
            spline.knots = [(i - 3) * 0.2 for i in range(11)]
            spline.yaw_pts = [0.0] * 7
            spline_pub.publish(spline)
            self.wait_for(lambda: any(e.trajectory_id == 1 and e.state == TrajectoryStatus.ACTIVE
                                      for e in executions),
                          'executor did not report the received return trajectory')
            self.wait_for(lambda: any(p.trajectory_id == 1 and not p.finished for p in points),
                          'bridge did not reopen for recovered return output')
            self.wait_for(lambda: statuses[-1].state == MissionStatus.RETURNING,
                          'executed trajectory did not resume return')
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
    rostest.rosrun('fuel_bridge', 'return_recovery_contract', ReturnRecoveryTest)
