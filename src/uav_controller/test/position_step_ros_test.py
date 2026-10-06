#!/usr/bin/env python3
"""Check OFFBOARD position steps through the real launch and ROS adapter."""
import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, ExtendedState, PositionTarget, State
from uav_controller.msg import FlightStatus, TrajectoryCommand


class PositionStepRosTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.messages = {name: [] for name in ('reference', 'status', 'attitude', 'command')}
        self.subscribers = [
            rospy.Subscriber(topic, kind, self.record, name)
            for name, topic, kind in (
                ('reference', '/uav_controller/active_setpoint', PositionTarget),
                ('status', '/uav_controller/fsm_status', FlightStatus),
                ('attitude', '/step_test_fcu/attitude', AttitudeTarget),
                ('command', '/uav_controller/trajectory_command', TrajectoryCommand))
        ]
        self.pose_pub = rospy.Publisher('/step_test_fcu/pose', PoseStamped, queue_size=5)
        self.velocity_pub = rospy.Publisher('/step_test_fcu/velocity', TwistStamped, queue_size=5)
        self.state_pub = rospy.Publisher('/step_test_fcu/state', State, queue_size=5)
        self.extended_pub = rospy.Publisher('/step_test_fcu/extended_state', ExtendedState, queue_size=5)
        self.position = (2.0, -1.0, 0.0)
        self.yaw = 0.4

    def record(self, msg, name):
        with self.lock:
            self.messages[name].append(msg)

    def samples(self, name):
        with self.lock:
            return list(self.messages[name])

    def pump(self, duration, armed, offboard, airborne):
        deadline = time.monotonic() + duration
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            stamp = rospy.Time.now()
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.position
            pose.pose.orientation.z = math.sin(self.yaw / 2.0)
            pose.pose.orientation.w = math.cos(self.yaw / 2.0)
            velocity = TwistStamped()
            velocity.header.stamp = stamp
            state = State()
            state.header.stamp = stamp
            state.connected, state.armed = True, armed
            state.mode = 'OFFBOARD' if offboard else 'POSCTL'
            extended = ExtendedState()
            extended.header.stamp = stamp
            extended.landed_state = (ExtendedState.LANDED_STATE_IN_AIR if airborne
                                     else ExtendedState.LANDED_STATE_ON_GROUND)
            self.pose_pub.publish(pose)
            self.velocity_pub.publish(velocity)
            self.extended_pub.publish(extended)
            self.state_pub.publish(state)
            rate.sleep()

    def assert_step(self, previous_transition):
        statuses = [s for s in self.samples('status')
                    if s.transition_id > previous_transition
                    and s.transition_reason == 'POSITION_STEP_STARTED']
        self.assertTrue(statuses, 'OFFBOARD entry did not start a position step')
        entry = statuses[0]
        self.assertEqual((entry.previous_phase, entry.phase, entry.mission_stage),
                         ('WAIT', 'HOLD', 'NONE'))
        references = [r for r in self.samples('reference') if r.header.stamp >= entry.header.stamp]
        self.assertGreater(len(references), 10)
        for reference in references:
            for axis, expected in zip(('x', 'y', 'z'), (0.0, 0.0, 1.0)):
                self.assertAlmostEqual(getattr(reference.position, axis), expected)
                self.assertEqual(getattr(reference.velocity, axis), 0.0)
                self.assertEqual(getattr(reference.acceleration_or_force, axis), 0.0)
            self.assertEqual(reference.yaw, 0.0)
        self.assertGreater(self.samples('attitude')[-1].thrust, 0.0)
        return entry.transition_id

    def test_manual_takeoff_and_repeatable_step(self):
        self.pump(1.0, armed=False, offboard=False, airborne=False)
        self.pump(0.4, armed=True, offboard=True, airborne=False)
        self.assertEqual(self.samples('status')[-1].phase, 'WAIT')
        self.assertEqual(self.samples('attitude')[-1].thrust, 0.0)

        self.position = (2.0, -1.0, 2.0)
        self.pump(0.5, armed=True, offboard=False, airborne=True)
        self.assertEqual(self.samples('status')[-1].phase, 'WAIT')
        reference = self.samples('reference')[-1]
        self.assertAlmostEqual(reference.position.x, 2.0)
        self.assertAlmostEqual(reference.position.z, 2.0)
        self.pump(0.5, armed=True, offboard=True, airborne=True)
        first_transition = self.assert_step(0)

        self.position = (-1.0, 0.8, 1.6)
        self.yaw = -0.6
        self.pump(0.4, armed=True, offboard=False, airborne=True)
        self.assertEqual(self.samples('status')[-1].phase, 'WAIT')
        self.pump(0.5, armed=True, offboard=True, airborne=True)
        self.assert_step(first_transition)
        self.assertTrue(all(s.phase in ('WAIT', 'HOLD') for s in self.samples('status')))
        commands = self.samples('command')
        self.assertTrue(commands)
        self.assertTrue(all(not command.active for command in commands))


if __name__ == '__main__':
    rospy.init_node('position_step_ros_test')
    rostest.rosrun('uav_controller', 'position_step_ros', PositionStepRosTest)
