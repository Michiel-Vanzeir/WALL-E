#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def motor_cb(data):
    rospy.loginfo("I heard %s", data.data)

def subscriber():
    rospy.init_node("motor_controller")
    rospy.Subscriber("motor_controls", String, motor_cb)
    rospy.spin()

if __name__ == "__main__":
    subscriber()

