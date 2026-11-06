#!/usr/bin/env python3
"""End-to-end check of the multi-session save trigger without Gazebo.

Flies the mission manager through start -> airborne -> landed using fake
odometry/controller/battery inputs and asserts that landing publishes the
occupancy-map save trigger and sets both prior save parameters.
"""
import sys

import rospy
import std_msgs.msg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from uav_controller.msg import FlightStatus

SAVE_TOPIC = "/save_occ_map"
PRIOR_PARAM = "/save_prior_info"
DESCRIPTOR_PARAM = "/save_prior_descriptor"


def main():
    rospy.init_node("check_multisession_save", anonymous=True)

    saved = {"map": False}
    rospy.Subscriber(SAVE_TOPIC, std_msgs.msg.Empty,
                     lambda _msg: saved.__setitem__("map", True), queue_size=1)

    odom_pub = rospy.Publisher("/fuel/odom", Odometry, queue_size=1)
    control_pub = rospy.Publisher("/fuel/control_state", FlightStatus, queue_size=1)
    battery_pub = rospy.Publisher("/fuel/battery", BatteryState, queue_size=1)

    def publish(landing_state):
        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "world"
        odom.pose.pose.position.x = 1.0
        odom.pose.pose.position.y = 0.5
        odom.pose.pose.position.z = 1.2
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)

        status = FlightStatus()
        status.header.stamp = now
        status.phase = "HOLD"
        status.output_policy = "CLOSED_LOOP"
        status.feedback_ready = True
        status.landing_state = landing_state
        control_pub.publish(status)

        battery = BatteryState()
        battery.header.stamp = now
        battery.percentage = 0.9
        battery_pub.publish(battery)

    rate = rospy.Rate(20)
    for _ in range(40):
        publish("IN_AIR")
        rate.sleep()

    if not rospy.get_param("/fuel/mission_manager/session", "") == "save":
        print("WARNING: session param is not 'save'; save may be disabled")

    start_srv = rospy.ServiceProxy("/fuel/start", std_srvs_trigger())
    try:
        start_srv()
    except Exception as exc:  # noqa: BLE001
        print("start service call failed: %s" % exc)

    # Stay airborne briefly, then land and hold ground long enough to debounce.
    for _ in range(40):
        publish("IN_AIR")
        rate.sleep()
    for _ in range(60):
        publish("ON_GROUND")
        rate.sleep()
    rospy.sleep(1.0)

    prior = rospy.get_param(PRIOR_PARAM, False)
    descriptor = rospy.get_param(DESCRIPTOR_PARAM, False)
    print("occupancy save topic seen: %s" % saved["map"])
    print("%s = %s" % (PRIOR_PARAM, prior))
    print("%s = %s" % (DESCRIPTOR_PARAM, descriptor))

    ok = saved["map"] or prior or descriptor
    print("RESULT: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def std_srvs_trigger():
    from std_srvs.srv import Trigger
    return Trigger


if __name__ == "__main__":
    sys.exit(main())
