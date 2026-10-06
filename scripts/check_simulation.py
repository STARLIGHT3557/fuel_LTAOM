#!/usr/bin/env python3
"""Headless P600 mission check: explore, inject low battery, return, hold home.

Run through src/fuel_bridge/scripts/fuel_env.sh. The ``both`` scene option runs
cylinder and indoor4 sequentially with private ROS/Gazebo masters. Each run only
signals its own subprocess group and refuses to share an existing PX4/gzserver.
"""
import argparse
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def context_key(context):
    return (context.manager_epoch, context.mission_id, context.task_type, context.source_id)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', choices=('cylinder', 'indoor4', 'both'), default='both')
    parser.add_argument('--explore-seconds', type=float, default=45,
                        help='Maximum exploration time before injecting 29%% battery.')
    parser.add_argument('--return-timeout', type=float, default=180)
    return parser.parse_args()


def run_both(args):
    for scene in ('cylinder', 'indoor4'):
        command = [sys.executable, str(Path(__file__).resolve()), '--scene', scene,
                   '--explore-seconds', str(args.explore_seconds),
                   '--return-timeout', str(args.return_timeout)]
        print('=== Running full mission in {} ==='.format(scene), flush=True)
        subprocess.run(command, check=True)


def run_scene(args):
    for name in ('px4', 'gzserver'):
        if subprocess.run(['pgrep', '-x', name], stdout=subprocess.DEVNULL).returncode == 0:
            raise SystemExit('Existing {} detected; close that simulation before this check.'.format(name))
    root = Path(__file__).resolve().parent.parent
    work = Path(tempfile.mkdtemp(prefix='fuel-{}-check.'.format(args.scene)))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:{}'.format(free_port()),
                      GAZEBO_MASTER_URI='http://127.0.0.1:{}'.format(free_port()),
                      ROS_HOME=str(work / 'home'), ROS_LOG_DIR=str(work / 'log'),
                      FUEL_TSP_DIR=str(work / 'tsp'), ROS_IP='127.0.0.1',
                      GAZEBO_MODEL_DATABASE_URI='')
    os.environ.pop('ROS_HOSTNAME', None)
    for sub in ('home', 'log', 'tsp'):
        (work / sub).mkdir()
    print('Simulation logs: {}'.format(work), flush=True)
    log = (work / 'simulation.log').open('w')
    process = subprocess.Popen(
        ['bash', str(root / 'fuel_simulation.sh'), 'gui:=false', 'rviz:=false',
         'scene:={}'.format(args.scene), 'battery_topic:=/fuel_test/battery'],
        env=os.environ.copy(), stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    battery_timer = None
    try:
        import rospy
        import rosgraph
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import State
        from mavros_msgs.srv import CommandBool, SetMode
        from nav_msgs.msg import Odometry
        from sensor_msgs import point_cloud2
        from sensor_msgs.msg import BatteryState, PointCloud2
        from std_msgs.msg import Header
        from std_srvs.srv import Trigger
        from uav_controller.msg import FlightStatus, TrajectorySetpoint
        from uav_mission_msgs.msg import MissionStatus, TaskContext
        from visualization_msgs.msg import Marker

        def wait(predicate, timeout, description):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                if process.poll() is not None:
                    raise RuntimeError('Simulation exited; see {}'.format(work / 'simulation.log'))
                if predicate():
                    print(description, flush=True)
                    return
                time.sleep(0.1)
            raise RuntimeError('Timeout: ' + description)

        wait(rosgraph.is_master_online, 30, 'ROS master ready')

        def simulated_time_configured():
            result = subprocess.run(
                ['rosparam', 'get', '/use_sim_time'], env=os.environ.copy(),
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
            return result.returncode == 0 and result.stdout.strip().lower() == 'true'

        # rospy reads /use_sim_time only while initializing.  Starting this
        # checker in the short gap between roscore and Gazebo parameter loading
        # would otherwise stamp injected batteries in wall time and make every
        # sample invalid to the simulation-time mission manager.
        wait(simulated_time_configured, 30, 'ROS simulated time configured')
        rospy.init_node('fuel_simulation_check', disable_signals=True)
        latest = {}
        commands = []
        frontier_errors = []

        def remember(key, msg):
            latest[key] = msg

        def command(msg):
            commands.append((time.monotonic(), msg))

        def marker(msg):
            if msg.action == Marker.ADD and not msg.points:
                frontier_errors.append(msg)

        subscriptions = [
            rospy.Subscriber(topic, cls, lambda msg, key=key: remember(key, msg), queue_size=1)
            for key, topic, cls in [
                ('state', '/mavros/state', State),
                ('control', '/uav_controller/fsm_status', FlightStatus),
                ('odom', '/mavros/local_position/odom', Odometry),
                ('mapping', '/livox/lidar_mapping', PointCloud2),
                ('hits', '/fuel/cloud_hits_world', PointCloud2),
                ('mission', '/fuel/mission_status', MissionStatus),
            ]
        ]
        subscriptions += [
            rospy.Subscriber('/uav_controller/trajectory_setpoint', TrajectorySetpoint, command),
            rospy.Subscriber('/planning_vis/frontier', Marker, marker),
        ]
        battery_level = [0.31]
        battery_pub = rospy.Publisher('/fuel_test/battery', BatteryState, queue_size=2)

        def publish_battery(_):
            battery_pub.publish(BatteryState(header=Header(stamp=rospy.Time.now()),
                                             percentage=battery_level[0]))

        battery_timer = rospy.Timer(rospy.Duration(0.1), publish_battery)
        wait(lambda: battery_pub.get_num_connections() > 0, 30, 'Injected battery subscriber ready')
        wait(lambda: 'state' in latest and latest['state'].connected and
             all(key in latest for key in ('mapping', 'hits', 'odom', 'control', 'mission')) and
             latest['control'].feedback_ready and latest['mission'].battery_valid,
             120, 'MAVROS, mapping, controller, mission manager, and 31% battery ready')
        assert latest['mapping'].header.frame_id == 'lidar_link'
        assert latest['hits'].header.frame_id == 'world'
        ranges = [math.sqrt(x*x + y*y + z*z) for x, y, z in point_cloud2.read_points(
            latest['mapping'], field_names=('x', 'y', 'z'), skip_nans=True)]
        if args.scene == 'cylinder':
            assert any(abs(distance - 40) < 0.01 for distance in ranges), \
                'Open scene produced no preserved max-range mapping rays'
        else:
            # The enclosed indoor scene may legitimately return a wall hit in
            # every direction; it must still respect the configured range.
            assert ranges and max(ranges) <= 40.01, 'Indoor mapping range is invalid'
        assert any(0.6 < distance < 39.9 for distance in ranges), 'No real obstacle returns'

        set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        ready_at = rospy.Time.now() + rospy.Duration(2)
        wait(lambda: rospy.Time.now() >= ready_at, 30, 'Initial setpoint stream established')
        last_request = [0.0]

        def enter_offboard():
            if latest['state'].mode == 'OFFBOARD':
                return True
            if time.monotonic() - last_request[0] >= 1:
                set_mode(custom_mode='OFFBOARD')
                last_request[0] = time.monotonic()
            return False

        wait(enter_offboard, 30, 'OFFBOARD active')
        assert arm(value=True).success
        wait(lambda: latest['state'].armed, 15, 'Simulated P600 armed')
        wait(lambda: abs(latest['odom'].pose.pose.position.z - 1.0) < 0.25 and
             math.sqrt(sum(value*value for value in (
                 latest['odom'].twist.twist.linear.x,
                 latest['odom'].twist.twist.linear.y,
                 latest['odom'].twist.twist.linear.z))) < 0.25 and
             latest['control'].phase in ('HOLD', 'TRAJECTORY'),
             90, 'Takeoff and stable hover verified')

        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        wait(lambda: goal_pub.get_num_connections() > 0, 10, 'Goal subscriber ready')
        goal = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='world'))
        goal.pose.orientation.w = 1
        goal_pub.publish(goal)
        wait(lambda: latest['mission'].state == MissionStatus.EXPLORING,
             15, 'Mission manager entered EXPLORING')
        explore_context = context_key(latest['mission'].context)
        wait(lambda: sum(not msg.finished and context_key(msg.context) == explore_context
                         for _, msg in commands) > 10,
             60, 'Authorized FUEL exploration trajectory reached controller')

        start = [latest['odom'].pose.pose.position.x,
                 latest['odom'].pose.pose.position.y,
                 latest['odom'].pose.pose.position.z]
        home = [0.0, 0.0, 1.0]
        end = time.monotonic() + args.explore_seconds
        max_travel = 0.0
        while time.monotonic() < end:
            state = latest['state']
            odom = latest['odom']
            position = [odom.pose.pose.position.x, odom.pose.pose.position.y,
                        odom.pose.pose.position.z]
            assert state.armed and state.mode == 'OFFBOARD'
            assert latest['control'].feedback_ready
            assert latest['control'].output_policy == 'CLOSED_LOOP'
            assert 0.2 < position[2] < 3.0
            max_travel = max(max_travel, math.dist(start, position))
            if max_travel > 1.0 and math.dist(position, home) > 3.0:
                break
            time.sleep(0.1)
        assert max_travel > 0.5, 'Vehicle did not travel during exploration'
        assert not frontier_errors, 'Empty frontier ADD markers were published'

        return_triggered_at = time.monotonic()
        battery_level[0] = 0.29
        wait(lambda: latest['mission'].state in
             (MissionStatus.PREPARE_RETURN, MissionStatus.RETURNING, MissionStatus.RETURN_RECOVERY, MissionStatus.HOME_HOLD) and
             latest['mission'].return_reason == MissionStatus.LOW_BATTERY,
             10, '29% battery latched LOW_BATTERY return')
        return_context = context_key(latest['mission'].context)
        assert return_context != explore_context
        battery_level[0] = 0.31
        time.sleep(1.0)
        assert context_key(latest['mission'].context) == return_context
        assert latest['mission'].state != MissionStatus.EXPLORING
        assert not any(stamp > return_triggered_at + 0.5 and
                       context_key(msg.context) == explore_context and not msg.finished
                       for stamp, msg in commands), 'Old exploration setpoint survived return switch'

        try:
            wait(lambda: latest['mission'].state == MissionStatus.HOME_HOLD,
                 args.return_timeout, 'Return trajectory reached a terminal state')
        except RuntimeError:
            odom = latest['odom']
            velocity = odom.twist.twist.linear
            position = odom.pose.pose.position
            mission = latest['mission']
            print('Return timeout snapshot: state={} detail={!r} pos=[{:.3f}, {:.3f}, {:.3f}] '
                  'vel=[{:.3f}, {:.3f}, {:.3f}] odom_age={:.3f}s'.format(
                      mission.state, mission.detail, position.x, position.y, position.z,
                      velocity.x, velocity.y, velocity.z,
                      (rospy.Time.now() - odom.header.stamp).to_sec()), flush=True)
            raise
        print('Return trajectory reached stable HOME_HOLD', flush=True)
        wait(lambda: commands and commands[-1][1].finished and
             context_key(commands[-1][1].context) == return_context,
             2, 'Home hold setpoint received')
        previous, held_command = commands[-2][1], commands[-1][1]
        assert context_key(previous.context) == return_context
        for field in ('position', 'velocity', 'acceleration'):
            before, after = getattr(previous, field), getattr(held_command, field)
            assert math.dist([before.x, before.y, before.z], [after.x, after.y, after.z]) < 1e-3, \
                'Home hold introduced a {} reference jump'.format(field)
        yaw_jump = math.atan2(math.sin(held_command.yaw - previous.yaw),
                              math.cos(held_command.yaw - previous.yaw))
        assert abs(yaw_jump) < 1e-3, 'Home hold introduced a yaw reference jump'
        q = latest['odom'].pose.pose.orientation
        actual_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        assert abs(actual_yaw) <= 0.10, 'Home hold entered before yaw converged'
        odom = latest['odom']
        final_position = [odom.pose.pose.position.x, odom.pose.pose.position.y,
                          odom.pose.pose.position.z]
        final_speed = math.sqrt(sum(value*value for value in (
            odom.twist.twist.linear.x, odom.twist.twist.linear.y,
            odom.twist.twist.linear.z)))
        assert math.dist(final_position, home) <= 0.25
        assert final_speed <= 0.20
        assert latest['mission'].return_reason == MissionStatus.LOW_BATTERY
        return_ids = {msg.trajectory_id for _, msg in commands
                      if context_key(msg.context) == return_context and not msg.finished}
        assert return_ids, 'No authorized return trajectory reached the controller'
        return_distance = math.dist(final_position, home)
        held = list(final_position)
        time.sleep(3)
        now_position = [latest['odom'].pose.pose.position.x,
                        latest['odom'].pose.pose.position.y,
                        latest['odom'].pose.pose.position.z]
        assert math.dist(held, now_position) < 0.35, 'Home hold drifted after mission completion'
        assert context_key(latest['mission'].context) == return_context

        stop = rospy.ServiceProxy('/fuel/stop', Trigger)
        assert stop().success
        print('PASS [{}]: explored {:.2f} m, low-battery return used {} trajectory IDs, '
              'home=[{:.2f}, {:.2f}, {:.2f}], then held. Logs: {}'.format(
                  args.scene, max_travel, len(return_ids), *final_position, work), flush=True)
    finally:
        if battery_timer is not None:
            battery_timer.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        log.close()


def main():
    args = parse_args()
    if args.scene == 'both':
        run_both(args)
    else:
        run_scene(args)


if __name__ == '__main__':
    main()
