#!/usr/bin/env python3
"""Delayed exploration handovers retain continuity checks and acknowledgement timeout."""
import time
import unittest

import rospy
import rostest
from bspline.msg import Bspline
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from uav_mission_msgs.msg import ControllerAck, TaskContext, TrajectoryStatus


class ExplorationHandoverTest(unittest.TestCase):
    def wait_for(self, predicate, message, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        self.fail(message)

    def test_delayed_handover(self):
        events, commands = [], []
        context = TaskContext(manager_epoch=123, mission_id=1,
                              task_type=TaskContext.EXPLORE, source_id='fuel_planner')
        auth = rospy.Publisher('/fuel/authorized_context', TaskContext, queue_size=1, latch=True)
        odom_pub = rospy.Publisher('/odom_world', Odometry, queue_size=2)
        spline_pub = rospy.Publisher('/planning/bspline', Bspline, queue_size=2)
        ack_pub = rospy.Publisher('/fuel/controller_ack', ControllerAck, queue_size=2)
        subscribers = [rospy.Subscriber('/fuel/trajectory_status', TrajectoryStatus, events.append),
                       rospy.Subscriber('/position_cmd', PositionCommand, commands.append)]

        def feed(_):
            msg = Odometry()
            msg.header.stamp = rospy.Time.now()
            msg.pose.pose.position.z = 1.0
            msg.pose.pose.orientation.w = 1.0
            odom_pub.publish(msg)

        def send(identifier, delay, x=0.0):
            msg = Bspline(context=context, order=3, traj_id=identifier,
                          start_time=rospy.Time.now() + rospy.Duration(delay), yaw_dt=0.5)
            msg.pos_pts = [Point(x, 0.0, 1.0) for _ in range(7)]
            msg.knots = [(i-3)*0.5 for i in range(11)]
            msg.yaw_pts = [0.0]*7
            spline_pub.publish(msg)

        def seen(identifier, state):
            return any(e.context == context and e.trajectory_id == identifier and
                       e.state == state for e in events)

        def ack(identifier):
            msg = ControllerAck(context=context, trajectory_id=identifier, accepted=True)
            msg.header.stamp = rospy.Time.now()
            ack_pub.publish(msg)

        timer = rospy.Timer(rospy.Duration(0.02), feed)
        try:
            self.wait_for(lambda: auth.get_num_connections() and spline_pub.get_num_connections()
                          and ack_pub.get_num_connections(), 'executor did not connect')
            auth.publish(context)
            time.sleep(0.15)
            # Planning completed after the nominal switch: a continuous candidate
            # must still execute even though the start timestamp is already past.
            send(1, -0.25)
            self.wait_for(lambda: seen(1, TrajectoryStatus.PENDING), 'late exploration was rejected')
            ack(1)
            self.wait_for(lambda: seen(1, TrajectoryStatus.ACTIVE), 'late exploration did not activate')

            send(2, 0.1)
            self.wait_for(lambda: seen(2, TrajectoryStatus.PENDING), 'replacement was not pending')
            time.sleep(0.3)
            ack(2)
            self.wait_for(lambda: seen(2, TrajectoryStatus.ACTIVE), 'delayed confirmation was rejected')

            # Removing the schedule deadline must not admit spatial discontinuities.
            send(3, -0.25, x=3.0)
            self.wait_for(lambda: seen(3, TrajectoryStatus.REJECTED), 'discontinuous candidate accepted')
            self.assertFalse(seen(3, TrajectoryStatus.ACTIVE))

            send(4, 0.1)
            self.wait_for(lambda: seen(4, TrajectoryStatus.PENDING), 'unacknowledged candidate not pending')
            self.wait_for(lambda: seen(4, TrajectoryStatus.REJECTED), 'acknowledgement timeout removed')
            self.assertTrue(any(e.trajectory_id == 4 and 'confirmation timed out' in e.detail
                                for e in events))
            marker = len(commands)
            self.wait_for(lambda: len(commands) > marker+3, 'safe active output interrupted')
            self.assertTrue(all(c.trajectory_id == 2 for c in commands[marker:]))

            # Return-home keeps the strict fixed-time handover policy.
            context = TaskContext(manager_epoch=123, mission_id=2,
                                  task_type=TaskContext.RETURN_HOME, source_id='fuel_planner')
            auth.publish(context)
            time.sleep(0.1)
            send(1, -0.25)
            self.wait_for(lambda: seen(1, TrajectoryStatus.REJECTED), 'expired return was accepted')
            send(2, 0.1)
            self.wait_for(lambda: seen(2, TrajectoryStatus.PENDING), 'return was not pending')
            self.wait_for(lambda: seen(2, TrajectoryStatus.REJECTED), 'return ack deadline removed')
            self.assertFalse(seen(2, TrajectoryStatus.ACTIVE))
        finally:
            timer.shutdown()


if __name__ == '__main__':
    rospy.init_node('test_exploration_handover')
    rostest.rosrun('prometheus_fuel', 'exploration_handover', ExplorationHandoverTest)
