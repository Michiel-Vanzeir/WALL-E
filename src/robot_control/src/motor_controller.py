#! /usr/bin/env python3
import rospy
from robot_control.msg import motor_cmd
from adafruit_motorkit import MotorKit

kit = MotorKit(i2c=board.I2C())

def motor_controller(data):
    kit.motor1.throttle(data.left_motor)
    kit.motor2.throttle(data.right_motor)

def subsriber():
    rospy.init_node("motor_controller")
    rospy.subscriber("motor_commands", motor_cmd, motor_controller)
    rospy.spin()
