#! /usr/bin/env python3
import rospy
import time
from computer_vision.msg import motor_throttle
from adafruit_motorkit import MotorKit


kit = MotorKit()


def controlMotors(data):
    kit.motor1.throttle = data.left_motor
    kit.motor2.throttle = data.right_motor



def subscriber():
    # Initialize node and subscriber
    rospy.init_node('motor_controller')
    rospy.Subscriber('motor_controls', motor_throttle, controlMotors)

    rospy.spin()


if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
