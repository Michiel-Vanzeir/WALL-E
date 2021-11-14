#! /usr/bin/env python3
from adafruit_motorkit import MotorKit
from computer_vision.msg import motor_throttle
import rospy
import time

kit = MotorKit()

def controlMotors(data):
    kit.motor1.throttle = data.left_motor
    kit.motor2.throttle = data.right_motor


def subscriber():
    # Initialize node and subscriber
    rospy.init_node('motor_controller')
    rospy.Subscriber('throttle_feed', motor_throttle, controlMotors)

    rospy.spin()


if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        rospy.loginfo('Motor controller node terminated.')
