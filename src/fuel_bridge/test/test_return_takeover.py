#!/usr/bin/env python3
"""Real FUEL: re-entering OFFBOARD during a return must not chase FUEL's advanced reference."""
import copy
import math
import time
import unittest

import rospy
import rostest
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import BatteryState, PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from uav_controller.msg import FlightStatus, TrajectorySetpoint
from uav_mission_msgs.msg import MissionStatus

LIDAR_Z, LIDAR_PITCH = 0.2769686356, 0.35
START = (0.0, 0.0, 1.5)
HOME = (3.0, 0.0, 1.5)
# Longer than most of the 9 s trajectory home, so FUEL's reference ends up well past the limit.
TAKEOVER_TIME = 6.0
REFERENCE_JUMP_LIMIT = 0.75


class ReturnTakeoverTest(unittest.TestCase):
    def wait_for(self, predicate, message, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.05)
        self.fail(message)

    def test_resume_from_aircraft_position(self):
        statuses, commands, maps = [], [], []
        subscribers = [
            rospy.Subscriber('/fuel/mission_status', MissionStatus, statuses.append),
            rospy.Subscriber('/uav_controller/trajectory_setpoint', TrajectorySetpoint, commands.append),
            rospy.Subscriber('/sdf_map/occupancy_all', PointCloud2, maps.append),
        ]
        odom_pub = rospy.Publisher('/fuel_test/odom', Odometry, queue_size=2)
        cloud_pub = rospy.Publisher('/fuel_test/cloud', PointCloud2, queue_size=2)
        state_pub = rospy.Publisher('/uav_controller/fsm_status', FlightStatus, queue_size=2)
        battery_pub = rospy.Publisher('/fuel_test/battery', BatteryState, queue_size=2)
        pitch_c, pitch_s = math.cos(LIDAR_PITCH), math.sin(LIDAR_PITCH)
        directions = []
        for az in range(0, 360, 2):
            for el in range(-7, 54, 2):
                azimuth, elevation = math.radians(az), math.radians(el)
                x = math.cos(elevation) * math.cos(azimuth)
                y = math.cos(elevation) * math.sin(azimuth)
                z = math.sin(elevation)
                directions.append((x, y, z, -pitch_s*x + pitch_c*z))

        began = time.monotonic()
        phase = ['HOLD']
        position = [START[0], START[1], 0.2]

        def feed(_):
            if not commands:
                # Rising maps every height: the LiDAR barely sees below its own level.
                position[2] = min(START[2], 0.2 + 0.5 * (time.monotonic() - began))
            elif phase[0] == 'HOLD':
                # An ideal controller, so the aircraft really flies the commanded reference.
                target = commands[-1].position
                position[:] = [target.x, target.y, target.z]
            stamp = rospy.Time.now()
            odom = Odometry(header=Header(stamp=stamp, frame_id='world'))
            odom.pose.pose.position.x = position[0]
            odom.pose.pose.position.y = position[1]
            odom.pose.pose.position.z = position[2]
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            state_pub.publish(FlightStatus(feedback_ready=True, output_policy='CLOSED_LOOP', phase=phase[0]))
            battery_pub.publish(BatteryState(header=Header(stamp=stamp), percentage=0.31))
            points = []
            for x, y, z, dz in directions:
                distance = 12.0
                if dz < -1e-6:
                    floor_distance = (0.25 - position[2] - LIDAR_Z) / dz
                    if floor_distance > 0:
                        distance = min(distance, floor_distance)
                points.append((distance*x, distance*y, distance*z))
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id='lidar_link'), points))

        timer = rospy.Timer(rospy.Duration(0.1), feed)
        try:
            rospy.wait_for_service('/fuel/return_home', timeout=15)
            self.wait_for(lambda: time.monotonic()-began > 5 and statuses and statuses[-1].battery_valid
                          and any(m.width for m in maps), 'home was not mapped')
            self.assertTrue(rospy.ServiceProxy('/fuel/return_home', Trigger)().success)
            self.wait_for(lambda: statuses[-1].return_reason == MissionStatus.MANUAL_RETURN,
                          'return did not start')
            context = copy.deepcopy(statuses[-1].context)
            self.wait_for(lambda: any(c.context == context and not c.finished for c in commands),
                          'return produced no commands')

            # The pilot takes over. FUEL keeps advancing its trajectory meanwhile.
            phase[0] = 'WAIT'
            time.sleep(TAKEOVER_TIME)
            resume_from = list(position)
            resume_index = len(commands)
            phase[0] = 'HOLD'
            self.wait_for(lambda: len(commands) > resume_index, 'return did not resume after takeover')
            resumed = commands[resume_index].position
            self.assertLessEqual(math.dist((resumed.x, resumed.y, resumed.z), resume_from),
                                 REFERENCE_JUMP_LIMIT, 'reference jumped when OFFBOARD was re-entered')

            self.wait_for(lambda: statuses[-1].state == MissionStatus.HOME_HOLD,
                          'aircraft did not get home after the takeover', 60)
            self.assertEqual(statuses[-1].context, context)
            self.assertLess(math.dist(position, HOME), 0.25)
            self.assertTrue(rospy.ServiceProxy('/fuel/stop', Trigger)().success)
        finally:
            timer.shutdown()


if __name__ == '__main__':
    rospy.init_node('test_return_takeover')
    rostest.rosrun('fuel_bridge', 'return_takeover', ReturnTakeoverTest)
