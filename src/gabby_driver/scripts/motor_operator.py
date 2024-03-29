#!/usr/bin/env python3
import rospy
import time
from adafruit_motorkit import MotorKit
from gabby_msgs.msg import Throttle

kit = MotorKit()

def callback(msg):
    # Make sure the throttle value is in the allowed range
    if msg.right_throttle > 1:
        kit.motor1.throttle = 1
    elif msg.left_throttle > 1:
        kit.motor2.throttle = 1
    else:
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
