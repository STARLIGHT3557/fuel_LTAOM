#!/usr/bin/env python3
"""Mission watchdogs and executor-owned return activation/completion."""
import copy
import time
import unittest

import rospy
import rostest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from uav_controller.msg import FlightStatus
from uav_mission_msgs.msg import (ControllerAck, MissionCommand, MissionStatus,
                                 PlannerStatus, TrajectoryStatus)


class ExplorationTimeoutsTest(unittest.TestCase):
    def wait_for(self, predicate, message, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        self.fail(message)

    def test_guards(self):
        statuses = []
        subscriber = rospy.Subscriber('/fuel/mission_status', MissionStatus, statuses.append)
        odom_pub = rospy.Publisher('/fuel/odom', Odometry, queue_size=2)
        state_pub = rospy.Publisher('/fuel/control_state', FlightStatus, queue_size=2)
        battery_pub = rospy.Publisher('/fuel/battery', BatteryState, queue_size=2)
        planner_pub = rospy.Publisher('/fuel/planner_status', PlannerStatus, queue_size=2)
        current = {'context': None, 'planner_state': None, 'battery': 0.8}

        def feed(_):
            stamp = rospy.Time.now()
            odom = Odometry(header=Header(stamp=stamp, frame_id='world'))
            odom.pose.pose.position.z = 1.0
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            state_pub.publish(FlightStatus(header=Header(stamp=stamp), feedback_ready=True,
                                           output_policy='CLOSED_LOOP', phase='HOLD'))
            battery_pub.publish(BatteryState(header=Header(stamp=stamp), percentage=current['battery']))
            context, state = current['context'], current['planner_state']
            if context is not None and state is not None:
                planner_pub.publish(PlannerStatus(header=Header(stamp=stamp), context=context,
                                                  state=state, consecutive_failures=100))

        timer = rospy.Timer(rospy.Duration(0.03), feed)
        try:
            rospy.wait_for_service('/fuel/start', timeout=10)
            start = rospy.ServiceProxy('/fuel/start', Trigger)
            stop = rospy.ServiceProxy('/fuel/stop', Trigger)
            self.wait_for(lambda: statuses and statuses[-1].battery_valid and
                          planner_pub.get_num_connections(), 'manager not ready')

            def begin():
                current['planner_state'] = None
                current['context'] = None
                current['battery'] = 0.8
                self.assertTrue(stop().success)
                self.wait_for(lambda: statuses[-1].state == MissionStatus.READY, 'stop did not reset task')
                time.sleep(0.1)
                response = start()
                self.assertTrue(response.success, response.message)
                self.wait_for(lambda: statuses[-1].state == MissionStatus.EXPLORING, 'start failed')
                current['context'] = copy.deepcopy(statuses[-1].context)

            def returned(reason):
                self.wait_for(lambda: statuses[-1].state == MissionStatus.PREPARE_RETURN,
                              'expected return guard did not fire')
                self.assertEqual(statuses[-1].return_reason, reason)

            begin()
            current['planner_state'] = PlannerStatus.FAILED
            time.sleep(0.3)
            self.assertEqual(statuses[-1].state, MissionStatus.EXPLORING,
                             'failure count unexpectedly requested return')
            returned(MissionStatus.EXPLORE_PLAN_TIMEOUT)

            begin()
            current['planner_state'] = PlannerStatus.TRAJECTORY_READY
            time.sleep(0.15)
            current['planner_state'] = None
            returned(MissionStatus.TRAJECTORY_TIMEOUT)

            begin()
            current['planner_state'] = PlannerStatus.OUTPUT_TIMEOUT
            returned(MissionStatus.TRAJECTORY_TIMEOUT)

            begin()
            current['planner_state'] = PlannerStatus.EXPLORATION_DONE
            returned(MissionStatus.EXPLORATION_DONE)

            begin()
            current['battery'] = 0.29
            returned(MissionStatus.LOW_BATTERY)
        finally:
            timer.shutdown()
            subscriber.unregister()

    def test_return_activation_and_watchdogs(self):
        statuses, commands = [], []
        subscribers = [
            rospy.Subscriber('/fuel/mission_status', MissionStatus, statuses.append),
            rospy.Subscriber('/fuel/mission_command', MissionCommand, commands.append),
        ]
        odom_pub = rospy.Publisher('/fuel/odom', Odometry, queue_size=2)
        state_pub = rospy.Publisher('/fuel/control_state', FlightStatus, queue_size=2)
        planner_pub = rospy.Publisher('/fuel/planner_status', PlannerStatus, queue_size=2)
        trajectory_pub = rospy.Publisher('/fuel/trajectory_status', TrajectoryStatus, queue_size=2)
        ack_pub = rospy.Publisher('/fuel/controller_ack', ControllerAck, queue_size=2)
        current = {'context': None, 'planner_state': None, 'trajectory_id': 0}

        def feed(_):
            stamp = rospy.Time.now()
            odom = Odometry(header=Header(stamp=stamp, frame_id='world'))
            odom.pose.pose.position.z = 1.0
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            state_pub.publish(FlightStatus(header=Header(stamp=stamp), feedback_ready=True,
                                           output_policy='CLOSED_LOOP', phase='HOLD'))
            context, state = current['context'], current['planner_state']
            if context is not None and state is not None:
                planner_pub.publish(PlannerStatus(header=Header(stamp=stamp), context=context,
                    trajectory_id=current['trajectory_id'], state=state))

        timer = rospy.Timer(rospy.Duration(0.03), feed)
        try:
            rospy.wait_for_service('/fuel/return_home', timeout=10)
            stop = rospy.ServiceProxy('/fuel/stop', Trigger)
            return_home = rospy.ServiceProxy('/fuel/return_home', Trigger)
            self.wait_for(lambda: statuses and odom_pub.get_num_connections() and
                          state_pub.get_num_connections() and planner_pub.get_num_connections() and
                          trajectory_pub.get_num_connections(), 'manager not connected')
            time.sleep(0.15)

            def begin():
                current.update(context=None, planner_state=None, trajectory_id=0)
                self.assertTrue(stop().success)
                self.wait_for(lambda: statuses[-1].state == MissionStatus.READY, 'stop failed')
                response = return_home()
                self.assertTrue(response.success, response.message)
                self.wait_for(lambda: statuses[-1].state == MissionStatus.PREPARE_RETURN,
                              'return did not start')
                current['context'] = copy.deepcopy(statuses[-1].context)
                return current['context']

            def activate(identifier, context=None):
                trajectory_pub.publish(TrajectoryStatus(header=Header(stamp=rospy.Time.now()),
                    context=context or current['context'], trajectory_id=identifier,
                    state=TrajectoryStatus.ACTIVE))

            def replan_since(index):
                return any(c.context == current['context'] and c.command == MissionCommand.REPLAN
                           for c in commands[index:])

            context = begin()
            current['planner_state'] = PlannerStatus.GOAL_REACHED
            stale = copy.deepcopy(context)
            stale.mission_id += 1
            activate(1, stale)
            activate(0)
            ack_pub.publish(ControllerAck(context=context, trajectory_id=1, accepted=True))
            time.sleep(0.3)
            self.assertEqual(statuses[-1].state, MissionStatus.PREPARE_RETURN,
                             'ACK, zero ID, stale task or unactivated GOAL_REACHED advanced return')
            current['planner_state'] = PlannerStatus.PLANNING
            first_attempt = len(commands)
            self.wait_for(lambda: replan_since(first_attempt),
                          'first activation timeout must request same-task recovery')
            self.assertEqual(statuses[-1].state, MissionStatus.RETURN_RECOVERY)

            current.update(planner_state=PlannerStatus.EXECUTING, trajectory_id=1)
            activate(1)
            self.wait_for(lambda: statuses[-1].state == MissionStatus.RETURNING, 'ACTIVE ignored')
            checkpoint = len(commands)
            status_checkpoint = len(statuses)
            time.sleep(1.6)  # Longer than the 1.2 s initial planning deadline.
            self.assertFalse(replan_since(checkpoint), 'normal execution reused the initial deadline')
            self.assertTrue(all(s.state == MissionStatus.RETURNING for s in statuses[status_checkpoint:]))

            # Execution still needs a live planner, even after the first ACTIVE.
            current['planner_state'] = None
            self.wait_for(lambda: replan_since(checkpoint), 'planner heartbeat watchdog was removed')
            self.assertEqual(statuses[-1].state, MissionStatus.RETURN_RECOVERY)
            current.update(planner_state=PlannerStatus.EXECUTING, trajectory_id=2)
            activate(2)
            self.wait_for(lambda: statuses[-1].state == MissionStatus.RETURNING, 'recovery ACTIVE ignored')
            checkpoint = len(commands)
            current['planner_state'] = PlannerStatus.OUTPUT_TIMEOUT
            self.wait_for(lambda: replan_since(checkpoint), 'output timeout recovery was removed')
            current.update(planner_state=PlannerStatus.EXECUTING, trajectory_id=3)
            time.sleep(0.1)  # Drain the preceding timeout status before activation.
            activate(3)
            self.wait_for(lambda: statuses[-1].state == MissionStatus.RETURNING, 'output recovery failed')

            # Old activation/goal messages cannot roll back the executing ID;
            # an ACK for a future candidate cannot replace it either.
            activate(2)
            ack_pub.publish(ControllerAck(context=context, trajectory_id=4, accepted=True))
            current.update(planner_state=PlannerStatus.GOAL_REACHED, trajectory_id=2)
            time.sleep(0.5)
            self.assertNotEqual(statuses[-1].state, MissionStatus.HOME_HOLD,
                                'old terminal reference completed the active task')
            current['trajectory_id'] = 3
            self.wait_for(lambda: statuses[-1].state == MissionStatus.HOME_HOLD,
                          'matching ACTIVE/GOAL_REACHED and stable home did not complete')

            # A new batch must re-enable the initial deadline.
            begin()
            current['planner_state'] = PlannerStatus.PLANNING
            checkpoint = len(commands)
            self.wait_for(lambda: replan_since(checkpoint), 'new task did not reset activation timeout')

            # ACTIVE without any planner heartbeat must not disable all watchdogs.
            begin()
            activate(1)
            self.wait_for(lambda: statuses[-1].state == MissionStatus.RETURNING, 'ACTIVE ignored')
            checkpoint = len(commands)
            self.wait_for(lambda: replan_since(checkpoint), 'missing first planner heartbeat was ignored')
        finally:
            timer.shutdown()
            for subscriber in subscribers:
                subscriber.unregister()


if __name__ == '__main__':
    rospy.init_node('test_exploration_timeouts')
    rostest.rosrun('prometheus_fuel', 'exploration_timeouts', ExplorationTimeoutsTest)
