#!/usr/bin/env python3
import rospy
from pynput.keyboard import Listener

from gabby_msgs.msg import Throttle

operate = True

def on_press(key):
    global operate
    # Get the key press as a string and remove the quotes
    key_str = str(key).replace("'", "")
    if not operate:
        if key_str == "o":
            operate = True
        return

    # Initialize message and set default values
    msg = Throttle()
    msg.left_throttle = 0
    msg.right_throttle = 0
    
    # Set throttle values based on key press
    if key_str == "z":
        # Forward
        msg.left_throttle = 0.5
        msg.right_throttle = 0.5
    elif key_str == "q":
        # Left
        msg.left_throttle = 0
        msg.right_throttle = 0.5
    elif key_str == "d":
        # Right
        msg.left_throttle = 0.5
        msg.right_throttle = 0
    elif key_str == "s":
        # Backward
        msg.left_throttle = -0.5
        msg.right_throttle = -0.5
    elif key_str == "i":
        # Increase throttle
        msg.left_throttle = 0.5 * 1.1
        msg.right_throttle = 0.5 * 1.1
    elif key_str == "k":
        # Decrease throttle
        msg.left_throttle = 0.5 * 0.9
        msg.right_throttle = 0.5 * 0.9
    # If spacebar stop
    elif key_str == "Key.space":
        msg.left_throttle = 0
        msg.right_throttle = 0
    
    # Publish the message
    pub.publish(msg)

def main():
    # Initialize the ROS node
    rospy.init_node('teleoperator')
    
    # Create a publisher for the throttle message
    global pub
    pub = rospy.Publisher('/throttlefeed', Throttle, queue_size=2)
    
    # Start listening for key presses
    with Listener(on_press=on_press) as l:
        l.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down teleoperator node")