#! /usr/bin/env python3
import rospy
import time
from std_msgs.msg import String
from adafruit_motorkit import MotorKit


kit = MotorKit()


def controlMotors(msg):
    # Retrieve the wanted speed from the msg
    msg = msg.data.split('|')
    left_speed = float(msg[0])
    right_speed = float(msg[1])

    kit.motor1.throttle = left_speed
    kit.motor2.throttle = right_speed
    time.sleep(0.15)
    kit.motor1.throttle = 0
    kit.motor2.throttle = 0


def subscriber():
    # Initialize node and subscriber
    rospy.init_node('motor_controller')
    rospy.Subscriber('motor_controls', String, controlMotors)

    rospy.spin()


if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
