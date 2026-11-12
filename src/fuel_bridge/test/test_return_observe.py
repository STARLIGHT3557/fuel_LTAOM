#!/usr/bin/env python3
"""Real FUEL: a blocked home is approached and faced, so that a stale map can clear."""
import copy
import math
import time
import unittest

import rospy
import rostest
from bspline.msg import Bspline
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import BatteryState, PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from uav_controller.msg import FlightStatus, TrajectorySetpoint
from uav_mission_msgs.msg import MissionStatus, PlannerStatus

LIDAR_X, LIDAR_Z, LIDAR_PITCH = 0.1471448904, 0.2769686356, 0.35
HOME = (3.0, 0.0, 1.0)
WALL_HALF_WIDTH, FLOOR_Z = 1.5, 0.25
OBSERVE_RANGE = 5.0  # fast_exploration_manager.cpp
# More than max_ray_length (10 m) from the wall, so rays from here cannot clear it.
FAR_START = (-8.0, 0.0, 1.5)
# Close enough to map the wall, but still outside the observation range.
NEAR_START = (-4.0, 0.0, 1.5)
MAPPING_LEG_TIME = 3.5


def wrapped(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def crosses_stale_wall(origin, ray, length):
    """Whether a unit ray enters the volume of the wall before it ends."""
    near, far = 0.0, length
    low = (HOME[0] - 0.4, HOME[1] - WALL_HALF_WIDTH - 0.2, 0.0)
    high = (HOME[0] + 0.4, HOME[1] + WALL_HALF_WIDTH + 0.2, 3.0)
    for axis in range(3):
        if abs(ray[axis]) < 1e-9:
            if not low[axis] <= origin[axis] <= high[axis]:
                return False
            continue
        first = (low[axis] - origin[axis]) / ray[axis]
        second = (high[axis] - origin[axis]) / ray[axis]
        near, far = max(near, min(first, second)), min(far, max(first, second))
        if near > far:
            return False
    return True


class ReturnObserveTest(unittest.TestCase):
    def wait_for(self, predicate, message, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.05)
        self.fail(message)

    def test_blocked_home(self):
        statuses, commands, maps, splines, planning = [], [], [], [], []
        subscribers = [
            rospy.Subscriber('/fuel/mission_status', MissionStatus, statuses.append),
            rospy.Subscriber('/fuel/planner_status', PlannerStatus, planning.append),
            rospy.Subscriber('/uav_controller/trajectory_setpoint', TrajectorySetpoint, commands.append),
            rospy.Subscriber('/sdf_map/occupancy_all', PointCloud2, maps.append),
            rospy.Subscriber('/planning/bspline', Bspline, splines.append),
        ]
        odom_pub = rospy.Publisher('/fuel_test/odom', Odometry, queue_size=2)
        cloud_pub = rospy.Publisher('/fuel_test/cloud', PointCloud2, queue_size=2)
        state_pub = rospy.Publisher('/uav_controller/fsm_status', FlightStatus, queue_size=2)
        battery_pub = rospy.Publisher('/fuel_test/battery', BatteryState, queue_size=2)
        pitch_c, pitch_s = math.cos(LIDAR_PITCH), math.sin(LIDAR_PITCH)
        directions = []
        # 1 degree of azimuth keeps a 0.2 m map dense out to the 7 m between NEAR_START and home.
        for az in range(0, 360):
            for el in range(-7, 54, 2):
                azimuth, elevation = math.radians(az), math.radians(el)
                x = math.cos(elevation) * math.cos(azimuth)
                y = math.cos(elevation) * math.sin(azimuth)
                z = math.sin(elevation)
                directions.append((x, y, z, pitch_c*x + pitch_s*z, -pitch_s*x + pitch_c*z))

        began = time.monotonic()
        position = [FAR_START[0], FAR_START[1], 0.2]
        world = {'follow_from': None, 'yaw': 0.0, 'blocked': True, 'screen': None, 'silent': False}

        def feed(_):
            elapsed = time.monotonic() - began
            if world['follow_from'] is None:
                # Rise at both ends of the way home: the LiDAR barely sees below its own level.
                second_leg = elapsed >= MAPPING_LEG_TIME
                rise = elapsed - MAPPING_LEG_TIME if second_leg else elapsed
                position[:] = [NEAR_START[0] if second_leg else FAR_START[0], 0.0,
                               min(NEAR_START[2], 0.2 + 0.5 * rise)]
            elif len(commands) > world['follow_from']:
                # An ideal controller, so the aircraft really flies and turns as commanded.
                command = commands[-1]
                position[:] = [command.position.x, command.position.y, command.position.z]
                world['yaw'] = command.yaw
            stamp = rospy.Time.now()
            cos_yaw, sin_yaw = math.cos(world['yaw']), math.sin(world['yaw'])
            odom = Odometry(header=Header(stamp=stamp, frame_id='world'))
            odom.pose.pose.position.x = position[0]
            odom.pose.pose.position.y = position[1]
            odom.pose.pose.position.z = position[2]
            odom.pose.pose.orientation.z = math.sin(world['yaw'] / 2.0)
            odom.pose.pose.orientation.w = math.cos(world['yaw'] / 2.0)
            odom_pub.publish(odom)
            state_pub.publish(FlightStatus(feedback_ready=True, output_policy='CLOSED_LOOP',
                                           phase='TRAJECTORY'))
            battery_pub.publish(BatteryState(header=Header(stamp=stamp), percentage=0.31))
            sensor_x = position[0] + cos_yaw * LIDAR_X
            sensor_y = position[1] + sin_yaw * LIDAR_X
            sensor_z = position[2] + LIDAR_Z
            points = []
            for x, y, z, forward, up in directions:
                ray_x = cos_yaw * forward - sin_yaw * y
                ray_y = sin_yaw * forward + cos_yaw * y
                distance = 12.0
                if world['blocked'] and ray_x > 0.01:
                    wall = (HOME[0] - sensor_x) / ray_x
                    if wall > 0 and abs(sensor_y + wall*ray_y) < WALL_HALF_WIDTH and \
                            0.0 < sensor_z + wall*up < 3.0:
                        distance = min(distance, wall)
                if world['screen'] is not None and abs(ray_x) > 0.01:
                    sx, sy = world['screen']
                    hit = (sx - sensor_x) / ray_x
                    if hit > 0 and abs(sensor_y + hit * ray_y - sy) < 0.7:
                        distance = min(distance, hit)
                if up < -1e-6:
                    floor_distance = (FLOOR_Z - sensor_z) / up
                    if floor_distance > 0:
                        distance = min(distance, floor_distance)
                # Open sky behind the wall: such a ray returns nothing, so it updates no voxel.
                if world['silent'] and crosses_stale_wall((sensor_x, sensor_y, sensor_z),
                                                          (ray_x, ray_y, up), distance):
                    continue
                points.append((distance*x, distance*y, distance*z))
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id='lidar_link'), points))

        return_home = rospy.ServiceProxy('/fuel/return_home', Trigger)
        stop = rospy.ServiceProxy('/fuel/stop', Trigger)

        def wall_in_map():
            return bool(maps) and any(
                abs(x - HOME[0]) < 0.3 and abs(y - HOME[1]) < 0.6 and abs(z - HOME[2]) < 0.3
                for x, y, z in point_cloud2.read_points(maps[-1], ('x', 'y', 'z')))

        def begin_return():
            self.assertTrue(return_home().success)
            self.wait_for(lambda: statuses[-1].return_reason == MissionStatus.MANUAL_RETURN,
                          'return did not start')
            return copy.deepcopy(statuses[-1].context)

        def stop_mission():
            self.assertTrue(stop().success)
            self.wait_for(lambda: statuses[-1].state == MissionStatus.READY, 'stop failed')
            # Let the stop's hover command pass, so the next task starts from a known yaw.
            time.sleep(0.3)

        def finish_at_home(task, message):
            self.wait_for(lambda: statuses[-1].state == MissionStatus.HOME_HOLD, message, 70)
            self.assertEqual(statuses[-1].context, task)
            self.assertLess(math.dist(position, HOME), 0.25)
            stop_mission()

        def reported_blocked(task, since):
            return any(p.context == task and p.state == PlannerStatus.OBSERVING_HOME
                       for p in planning[since:])

        timer = rospy.Timer(rospy.Duration(0.1), feed)
        try:
            rospy.wait_for_service('/fuel/return_home', timeout=15)
            self.wait_for(lambda: time.monotonic()-began > 2*MAPPING_LEG_TIME and statuses
                          and statuses[-1].battery_valid and wall_in_map(),
                          'the way home and the wall were not mapped')

            # Stale map: the wall is gone, but from far away no ray can clear it. The aircraft has
            # to fly closer to find out, and then finishes at home as usual.
            world['follow_from'] = len(commands)
            position[:] = list(FAR_START)
            time.sleep(0.3)
            world['blocked'] = False
            time.sleep(1.0)
            self.assertTrue(wall_in_map(), 'rays from far away cleared the wall')
            task = begin_return()
            self.wait_for(lambda: any(s.context == task for s in splines),
                          'aircraft waited far away for a home that only looks blocked')
            first = next(s for s in splines if s.context == task).pos_pts[-1]
            self.assertGreater(math.dist((first.x, first.y, first.z), HOME), 1.0)
            finish_at_home(task, 'return did not come back to the cleared home')

            # The wall is really there: stop at the observation range and never report arrival.
            world.update(follow_from=len(commands), blocked=True)
            position[:] = list(NEAR_START)
            time.sleep(0.3)
            task = begin_return()
            self.wait_for(lambda: reported_blocked(task, 0), 'blocked home was not reported', 40)
            self.wait_for(lambda: statuses[-1].state == MissionStatus.RETURNING,
                          'healthy observation unexpectedly entered recovery')
            self.assertLess(math.dist(position, HOME), OBSERVE_RANGE + 0.2)
            self.assertGreater(math.dist(position, HOME), OBSERVE_RANGE - 1.5)
            first_view = list(position)
            checkpoint = len(planning)
            time.sleep(4.0)  # More than the no-observation timeout; the wall is still producing hits.
            self.assertLess(math.dist(position, first_view), 0.3)
            self.assertFalse(any(p.context == task and p.state in
                                 (PlannerStatus.FAILED, PlannerStatus.GOAL_REACHED)
                                 for p in planning[checkpoint:]))
            stop_mission()

            # Within range but with the tail towards home: turn on the spot before waiting.
            stopped = list(position)
            world.update(follow_from=len(commands), yaw=math.pi)
            time.sleep(0.3)
            checkpoint = len(planning)
            task = begin_return()
            self.wait_for(lambda: reported_blocked(task, checkpoint), 'aircraft did not turn and wait', 40)
            facing = math.atan2(HOME[1] - position[1], HOME[0] - position[0])
            self.assertLess(abs(wrapped(world['yaw'] - facing)), 0.5, 'aircraft did not face home')
            self.assertLess(math.dist(position, stopped), 0.3, 'turning moved the aircraft')

            # The wall leaves behind a nearer screen. From the first view its old voxels
            # cannot be cleared: the aircraft must find another reachable line of sight.
            eye_x = position[0] + math.cos(world['yaw']) * LIDAR_X
            eye_y = position[1] + math.sin(world['yaw']) * LIDAR_X
            world['screen'] = ((eye_x + HOME[0]) / 2, (eye_y + HOME[1]) / 2)
            time.sleep(1.0)
            stopped = list(position)
            world['blocked'] = False
            time.sleep(0.3)
            self.assertTrue(wall_in_map(), 'the screen did not mask the stale wall')
            self.wait_for(lambda: math.dist(position, stopped) > 0.8,
                          'aircraft stayed behind the screen', 40)
            finish_at_home(task, 'return did not continue once home cleared')

            # No ray ever comes back through the stale wall: after a few viewpoints the aircraft
            # stops relocating, reports the failure and waits where it is.
            world.update(follow_from=len(commands), screen=None, blocked=True)
            position[:] = list(NEAR_START)
            self.wait_for(wall_in_map, 'the wall was not mapped again')
            world.update(blocked=False, silent=True)
            time.sleep(0.5)
            self.assertTrue(wall_in_map(), 'rays cleared the wall behind which nothing returns')
            task = begin_return()
            self.wait_for(lambda: any(p.context == task and p.state == PlannerStatus.FAILED and
                                      'no new rays' in p.detail for p in planning),
                          'aircraft did not give up relocating', 90)
            self.wait_for(lambda: statuses[-1].state == MissionStatus.RETURN_RECOVERY,
                          'giving up was not reported as recovery')
            waiting_at = list(position)
            checkpoint = len(planning)
            time.sleep(8.0)  # Longer than a viewpoint attempt, so another move would show.
            self.assertLess(math.dist(position, waiting_at), 0.3, 'aircraft kept relocating')
            self.assertFalse(any(p.context == task and p.state == PlannerStatus.TRAJECTORY_READY
                                 for p in planning[checkpoint:]), 'aircraft kept planning new views')

            # Rays come back: the same task still finishes at home.
            world['silent'] = False
            finish_at_home(task, 'return did not continue after waiting')
        finally:
            timer.shutdown()
            for subscriber in subscribers:
                subscriber.unregister()


if __name__ == '__main__':
    rospy.init_node('test_return_observe')
    rostest.rosrun('fuel_bridge', 'return_observe', ReturnObserveTest)
