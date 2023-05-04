import pynput
import rospy
from gabby_msgs.msg import Throttle
from pynput.keyboard import Key, Listener

throttle = 0.35

def KeyPressed(key):
    global throttle
    commands = {Key.up: [throttle, throttle], 
                Key.down: [-throttle, -throttle], 
                Key.left: [-throttle, throttle],
                Key.right: [throttle, -throttle], 
                Key.space: (0, 0),
                Key.shift_l: [throttle*1.2, throttle*1.2],
                Key.shift_r: [throttle*0.8, throttle*0.8],}
    
    if key in commands:
        throttle_msg = Throttle()
        throttle_msg.left_throttle = commands[key][0]
        throttle_msg.right_throttle = commands[key][1]
        keyPub.publish(throttle_msg)


def Listen():
    with Listener(on_press=KeyPressed) as listener:
        listener.join()

if __name__ == '__main__':
    rospy.init_node('tele_operator')
    keyPub = rospy.Publisher('throttle', Throttle, queue_size=2)
    while not rospy.is_shutdown():
        Listen()
        