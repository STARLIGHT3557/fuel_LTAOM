#!/usr/bin/env python3
"""Exercise the real estimator/controller against a synthetic MAVROS interface."""
import math
import threading
import time
import unittest

import numpy as np
import rospy
import rostest
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, GPSRAW, PositionTarget, AttitudeTarget
from mavros_msgs.srv import SetMode, SetModeResponse
from prometheus_msgs.msg import UAVCommand, UAVControlState, UAVState
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry


class ControllerTest(unittest.TestCase):
    def test_flight_contract(self):
        ns = '/uav99'
        self.running = True
        self.armed = False
        self.mode = 'POSCTL'
        self.pose_enabled = True
        self.position = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.local = self.att = self.control = self.state = self.mesh = self.odom = None
        self.local_count = self.att_count = 0
        self.land_requests = []
        def mode_cb(req):
            self.land_requests.append(req.custom_mode)
            return SetModeResponse(mode_sent=True)
        service = rospy.Service(ns + '/mavros/set_mode', SetMode, mode_cb)
        pubs = {key: rospy.Publisher(ns + '/mavros/' + topic, cls, queue_size=10)
                for key, topic, cls in [
                    ('state', 'state', State), ('pose', 'local_position/pose', PoseStamped),
                    ('vel', 'local_position/velocity_local', TwistStamped),
                    ('imu', 'imu/data', Imu), ('gps', 'gpsstatus/gps1/raw', GPSRAW)]}
        cmd_pub = rospy.Publisher(ns + '/prometheus/command', UAVCommand, queue_size=10)
        def receive(name, msg):
            setattr(self, name, msg)
            if name in ('local', 'att'):
                setattr(self, name + '_count', getattr(self, name + '_count') + 1)
        subs = [rospy.Subscriber(ns + topic, cls, lambda m, n=name: receive(n, m))
                for name, topic, cls in [
                    ('local', '/mavros/setpoint_raw/local', PositionTarget),
                    ('att', '/mavros/setpoint_raw/attitude', AttitudeTarget),
                    ('control', '/prometheus/control_state', UAVControlState),
                    ('state', '/prometheus/state', UAVState),
                    ('mesh', '/prometheus/uav_mesh', Marker),
                    ('odom', '/prometheus/odom', Odometry)]]
        def feed():
            while self.running and not rospy.is_shutdown():
                stamp = rospy.Time.now()
                state = State(connected=True, armed=self.armed, mode=self.mode)
                pubs['state'].publish(state)
                pose = PoseStamped()
                pose.header.stamp = stamp
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.position
                pose.pose.orientation.w = 1
                if self.pose_enabled:
                    pubs['pose'].publish(pose)
                vel = TwistStamped()
                vel.header.stamp = stamp
                pubs['vel'].publish(vel)
                imu = Imu()
                imu.header.stamp = stamp
                yaw = self.yaw
                imu.orientation.z = math.sin(yaw / 2)
                imu.orientation.w = math.cos(yaw / 2)
                imu.linear_acceleration.z = 9.81
                pubs['imu'].publish(imu)
                pubs['gps'].publish(GPSRAW(fix_type=3, satellites_visible=12))
                time.sleep(0.01)
        thread = threading.Thread(target=feed, daemon=True)
        thread.start()
        def wait(predicate, timeout=5):
            end = time.monotonic() + timeout
            while time.monotonic() < end and not predicate():
                time.sleep(0.01)
            self.assertTrue(predicate(), 'Timed out waiting for controller condition')
        def send(cmd, duration=0.15):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                cmd.header.stamp = rospy.Time.now()
                cmd_pub.publish(cmd)
                time.sleep(0.01)
        try:
            wait(lambda: self.state is not None and self.state.odom_valid and self.local is not None)
            self.assertEqual(self.control.control_state, UAVControlState.INIT)
            wait(lambda: self.mesh is not None and self.odom is not None)
            self.assertEqual(self.odom.header.frame_id, 'world')
            self.assertEqual(self.mesh.mesh_resource, 'package://prometheus_uav_control/meshes/hummingbird.mesh')
            # Start on the ground, then take off using another flight mode.
            # OFFBOARD pre-streaming must follow the latest measured pose,
            # rather than retain the ground-level initialization target.
            self.armed = True
            self.position = [0.0, 0.0, 1.5]
            self.yaw = 0.4
            wait(lambda: abs(self.state.position[2] - 1.5) < 1e-5 and
                 abs(self.state.attitude[2] - 0.4) < 1e-5)
            time.sleep(0.1)
            self.assertAlmostEqual(self.local.position.z, 1.5, places=5,
                                   msg='OFFBOARD pre-stream retained the ground height')
            self.assertAlmostEqual(self.local.yaw, 0.4, places=5)
            self.mode = 'OFFBOARD'
            wait(lambda: self.control.control_state == UAVControlState.COMMAND_CONTROL)
            self.position = [0.2, 0.1, 1.7]
            self.yaw = 0.7
            wait(lambda: abs(self.state.position[2] - 1.7) < 1e-5)
            time.sleep(0.1)
            self.assertAlmostEqual(self.local.position.z, 1.5, places=5)
            self.assertAlmostEqual(self.local.yaw, 0.4, places=5)

            pos = UAVCommand(Agent_CMD=UAVCommand.Move, Move_mode=UAVCommand.XYZ_POS)
            pos.position_ref = [0, 0, 2]
            send(pos)
            self.assertAlmostEqual(self.local.position.z, 2)

            # Leave OFFBOARD with an explicit old target, move elsewhere, then
            # re-enter. Neither the old target nor the exit pose may be resumed.
            self.mode = 'POSCTL'
            wait(lambda: self.control.control_state == UAVControlState.INIT)
            self.position = [0.8, -0.5, 2.5]
            self.yaw = -0.6
            wait(lambda: abs(self.state.position[2] - 2.5) < 1e-5 and
                 abs(self.state.attitude[2] + 0.6) < 1e-5)
            send(pos)  # Movement commands outside OFFBOARD must not preload a target.
            self.assertAlmostEqual(self.local.position.z, 2.5, places=5)
            self.mode = 'OFFBOARD'
            wait(lambda: self.control.control_state == UAVControlState.COMMAND_CONTROL)
            self.assertAlmostEqual(self.local.position.x, 0.8, places=5)
            self.assertAlmostEqual(self.local.position.y, -0.5, places=5)
            self.assertAlmostEqual(self.local.position.z, 2.5, places=5)
            self.assertAlmostEqual(self.local.yaw, -0.6, places=5)

            # Return to the known pose for the numerical tracking checks below.
            self.mode = 'POSCTL'
            wait(lambda: self.control.control_state == UAVControlState.INIT)
            self.position = [0.0, 0.0, 1.5]
            self.yaw = 0.0
            wait(lambda: abs(self.state.position[2] - 1.5) < 1e-5 and
                 abs(self.state.attitude[2]) < 1e-5)
            self.mode = 'OFFBOARD'
            wait(lambda: self.control.control_state == UAVControlState.COMMAND_CONTROL)
            track = UAVCommand(Agent_CMD=UAVCommand.Move, Move_mode=UAVCommand.TRAJECTORY_ATTITUDE)
            track.position_ref = [0.1, 0.2, 1.8]
            track.velocity_ref = [0.1, -0.1, 0.2]
            track.acceleration_ref = [0.3, -0.2, 0.1]
            track.yaw_ref = 0.3
            send(track)
            wait(lambda: self.att is not None)
            # Known position/velocity errors and feedforward must produce the
            # same desired body z direction as the pre-refactor controller.
            desired = np.array([8, 8, 12]) * np.array([0.1, 0.2, 0.3])
            desired += np.array([4, 4, 5]) * np.array([0.1, -0.1, 0.2])
            desired += np.array([0.3, -0.2, 9.91])
            desired /= np.linalg.norm(desired)
            q = self.att.orientation
            actual_z = np.array([2*(q.x*q.z+q.w*q.y), 2*(q.y*q.z-q.w*q.x), 1-2*(q.x*q.x+q.y*q.y)])
            np.testing.assert_allclose(actual_z, desired, atol=2e-6)
            self.assertTrue(math.isfinite(self.att.thrust) and 0 < self.att.thrust <= 1)
            # A stopped trajectory holds one fixed position, including after drift.
            wait(lambda: self.local.header.stamp > self.att.header.stamp, timeout=3)
            self.position = [0.4, 0.2, 1.6]
            time.sleep(0.2)
            self.assertAlmostEqual(self.local.position.x, 0)
            self.assertAlmostEqual(self.local.position.z, 1.5)
            send(UAVCommand(Agent_CMD=UAVCommand.Current_Pos_Hover))
            self.assertAlmostEqual(self.local.position.x, 0.4, places=5)
            # Unused controller mode cannot change the target.
            pos.Move_mode = UAVCommand.TRAJECTORY
            send(pos)
            self.assertAlmostEqual(self.local.position.x, 0.4, places=5)
            # A healthy LAND stays LAND and never emits old trajectory setpoints.
            send(UAVCommand(Agent_CMD=UAVCommand.Land))
            wait(lambda: self.control.control_state == UAVControlState.LAND_CONTROL and 'AUTO.LAND' in self.land_requests)
            count = self.local_count + self.att_count
            time.sleep(0.2)
            self.assertEqual(count, self.local_count + self.att_count)
            # Reset only after disarm. Stale pose must invalidate odometry and
            # request landing again; the adapter must not republish stale odom.
            self.armed = False
            self.mode = 'POSCTL'
            wait(lambda: self.control.control_state == UAVControlState.INIT)
            self.armed = True
            self.mode = 'OFFBOARD'
            wait(lambda: self.control.control_state == UAVControlState.COMMAND_CONTROL)
            self.pose_enabled = False
            wait(lambda: not self.state.odom_valid and self.control.failsafe and self.control.control_state == UAVControlState.LAND_CONTROL)
            stamp = self.odom.header.stamp
            time.sleep(0.2)
            self.assertEqual(stamp, self.odom.header.stamp)
        finally:
            self.running = False
            thread.join(timeout=2)
            service.shutdown()


if __name__ == '__main__':
    rospy.init_node('p600_controller_test')
    rostest.rosrun('prometheus_uav_control', 'p600_controller', ControllerTest)
