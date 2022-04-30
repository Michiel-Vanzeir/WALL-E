#!/usr/bin/env python3
import rospy
import time
from adafruit_motorkit import MotorKit
from my_robot_msgs.msg import Throttle

kit = MotorKit()

def callback(msg):
    kit.motor1.throttle = msg.right_throttle
    kit.motor2.throttle = msg.left_throttle
    rospy.loginfo(f"Left throttle: {msg.left_throttle} || Right throttle: {msg.right_throttle}")
    
def motor_operator():
    rospy.init_node('motor_operator')
  
    rospy.Subscriber("throttlefeed", Throttle, callback)
  
    rospy.spin()
  
if __name__ == '__main__':
    try:
        motor_operator()
    except:
        kit.motor1.throttle = 0
        kit.motor2.throttle = 0
