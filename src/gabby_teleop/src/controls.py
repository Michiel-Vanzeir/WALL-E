#!/usr/bin/env python3
import rospy
from pynput.keyboard import Listener
from gabby_msgs.msg import Throttle

pub = rospy.Publisher('/throttlefeed', Throttle, queue_size=2)
throttle = 0.3 # Starting throttle

def doit(key):
    global throttle
    key = str(key).replace("'", "")
    msg = Throttle()
    
    if (key == "z"):
        msg.left_throttle = throttle
        msg.right_throttle = throttle
    elif (key == "q"):
        msg.left_throttle = 0
        msg.right_throttle = throttle
    elif (key == "d"):
        msg.left_throttle = throttle
        msg.right_throttle = 0
    elif (key == "s"):
        msg.left_throttle = -throttle
        msg.right_throttle = -throttle
    elif (key == "i"):
        # Increase throttle
        throttle *= 1.1
    elif (key == "k"):
        # Decrease throttle
        throttle *= 0.9
    else:
        msg.left_throttle = 0
        msg.right_throttle = 0
    
    pub.publish(msg)

def controls():
    rospy.init_node('teleoperator')
    
    # Listener for keystrokes
    with Listener(on_press=doit) as l:
        l.join()

if __name__ == '__main__':
    controls()
