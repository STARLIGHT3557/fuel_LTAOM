#!/usr/bin/env python3
"""Headless P600 smoke flight: take off, trigger through the RViz topic, stop.

Run through src/fuel_bridge/scripts/fuel_env.sh. Refuses to run alongside an
existing PX4 or gzserver; uses private ROS/Gazebo masters and only stops its
own subprocess group. This is a short dynamics check, not mission coverage.
"""
import argparse
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--explore-seconds', type=float, default=30)
    args = parser.parse_args()
    for name in ('px4', 'gzserver'):
        if subprocess.run(['pgrep', '-x', name], stdout=subprocess.DEVNULL).returncode == 0:
            raise SystemExit('Existing {} detected; close that simulation before this check.'.format(name))
    root = Path(__file__).resolve().parent.parent
    work = Path(tempfile.mkdtemp(prefix='fuel-simulation-check.'))
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
    process = subprocess.Popen(['bash', str(root / 'fuel_simulation.sh'), 'gui:=false', 'rviz:=false'],
                               env=os.environ.copy(), stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    try:
        import rospy
        import rosgraph
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.srv import CommandBool, SetMode
        from nav_msgs.msg import Odometry
        from prometheus_msgs.msg import UAVCommand, UAVState, UAVControlState
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs import point_cloud2
        from std_srvs.srv import Trigger
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
        rospy.init_node('fuel_simulation_check', disable_signals=True)
        latest = {}
        commands = []
        frontier_errors = []
        def remember(key, msg):
            latest[key] = msg
        def command(msg):
            if msg.Agent_CMD == UAVCommand.Move and msg.Move_mode == UAVCommand.TRAJECTORY_ATTITUDE:
                commands.append(msg)
        def marker(msg):
            if msg.action == Marker.ADD and not msg.points:
                frontier_errors.append(msg)
        subscriptions = [rospy.Subscriber(topic, cls, lambda msg, k=key: remember(k, msg), queue_size=1)
                         for key, topic, cls in [
                             ('state', '/uav1/prometheus/state', UAVState),
                             ('control', '/uav1/prometheus/control_state', UAVControlState),
                             ('odom', '/uav1/prometheus/odom', Odometry),
                             ('mapping', '/livox/lidar_mapping', PointCloud2),
                             ('hits', '/fuel/cloud_hits_world', PointCloud2),
                             ('mesh', '/uav1/prometheus/uav_mesh', Marker)]]
        subscriptions += [rospy.Subscriber('/uav1/prometheus/command', UAVCommand, command),
                          rospy.Subscriber('/planning_vis/frontier', Marker, marker)]
        wait(lambda: 'state' in latest and latest['state'].connected and latest['state'].odom_valid and
             all(k in latest for k in ('mapping', 'hits', 'mesh', 'odom')), 120, 'GPS, mapping cloud and UAV mesh ready')
        assert latest['mapping'].header.frame_id == 'uav1/lidar_link'
        assert latest['hits'].header.frame_id == 'world'
        ranges = [math.sqrt(x*x+y*y+z*z) for x,y,z in point_cloud2.read_points(
            latest['mapping'], field_names=('x', 'y', 'z'), skip_nans=True)]
        assert any(abs(r-40) < 0.01 for r in ranges), 'No max-range mapping rays'
        assert any(0.6 < r < 39.9 for r in ranges), 'No real obstacle returns'
        set_mode = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)
        arm = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)
        # PX4 needs a continuous setpoint stream before accepting OFFBOARD.
        # The first odometry/mesh callback can arrive before that interval.
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
        pub = rospy.Publisher('/uav1/prometheus/command', UAVCommand, queue_size=1)
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        wait(lambda: pub.get_num_connections() > 0 and goal_pub.get_num_connections() > 0, 10, 'Command/goal subscribers ready')
        cmd = UAVCommand(Agent_CMD=UAVCommand.Move, Move_mode=UAVCommand.XYZ_POS,
                         position_ref=[0, 0, 1.5], Command_ID=1)
        for _ in range(5):
            pub.publish(cmd)
            time.sleep(0.1)
        wait(lambda: abs(latest['state'].position[2] - 1.5) < 0.2 and
             math.sqrt(sum(v*v for v in latest['state'].velocity)) < 0.25 and
             latest['control'].control_state == UAVControlState.COMMAND_CONTROL, 90, 'Takeoff and stable hover verified')
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.header.stamp = rospy.Time.now()
        goal.pose.orientation.w = 1
        goal_pub.publish(goal)
        wait(lambda: len(commands) > 10, 60, 'RViz 2D Nav Goal topic triggered FUEL mode 9 commands')
        start = list(latest['state'].position)
        end = time.monotonic() + args.explore_seconds
        max_travel = 0
        while time.monotonic() < end:
            state = latest['state']
            assert state.armed and state.mode == 'OFFBOARD' and state.odom_valid
            assert not latest['control'].failsafe
            assert -1.2 <= state.position[0] <= 22.2 and -11.2 <= state.position[1] <= 10.2
            assert 0.2 < state.position[2] < 3.0
            max_travel = max(max_travel, math.dist(start, state.position))
            time.sleep(0.1)
        assert max_travel > 0.5, 'Vehicle did not travel during exploration'
        assert not frontier_errors, 'Empty frontier ADD markers were published'
        stop = rospy.ServiceProxy('/fuel/stop', Trigger)
        assert stop().success
        wait(lambda: math.sqrt(sum(v*v for v in latest['state'].velocity)) < 0.3, 30, 'Stop returned to hover')
        stopped = list(latest['state'].position)
        count = len(commands)
        time.sleep(3)
        assert len(commands) == count, 'Trajectory forwarded after stop'
        assert math.dist(stopped, latest['state'].position) < 0.5, 'Hover drift after stop'
        print('PASS: real mapping rays, takeoff, RViz trigger, {:.2f} m travel, stop/hover. {}'.format(max_travel, work), flush=True)
    finally:
        # Only the group created by this test is signalled.
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


if __name__ == '__main__':
    main()
