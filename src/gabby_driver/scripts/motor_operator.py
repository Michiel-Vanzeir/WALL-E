#!/usr/bin/env python3
import rospy

from gabby_driver.Motorshield import Motorshield
from gabby_msgs.msg import Throttle

# Initialize the motor shield
motorshield = Motorshield()

def limitThrottle(min_value, max_value, throttle):
    return min(max_value, max(min_value, throttle))

def throttleCallback(msg):
    # Limit the throttle values to the range [-1, 1]
    left_throttle = limitThrottle(-1, 1, msg.left_throttle)
    right_throttle = limitThrottle(-1, 1, msg.right_throttle)

    print(f"Left throttle: {left_throttle} || Right throttle: {right_throttle}")

    # Set the throttle values
    motorshield.setLeftThrottle(left_throttle)
    motorshield.setRightThrottle(right_throttle)

    # Log the throttle values
    rospy.loginfo(f"Left throttle: {msg.left_throttle} || Right throttle: {msg.right_throttle}")

def motor_operator():
    rospy.init_node("motor_operator")
  
    rospy.Subscriber("/throttlefeed", Throttle, throttleCallback)
    
    rospy.spin()
  
if __name__ == '__main__':
    try:
        motor_operator()
    except:
        motorshield.shutdown()
        rospy.error("Shutting down motor_operator node")
