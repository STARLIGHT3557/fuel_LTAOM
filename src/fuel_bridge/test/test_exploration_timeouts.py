#!/usr/bin/env python3
"""Removing count-based returns preserves the existing mission completion and watchdogs."""
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
from uav_mission_msgs.msg import MissionStatus, PlannerStatus


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


if __name__ == '__main__':
    rospy.init_node('test_exploration_timeouts')
    rostest.rosrun('prometheus_fuel', 'exploration_timeouts', ExplorationTimeoutsTest)
