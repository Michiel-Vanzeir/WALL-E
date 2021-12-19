#! /usr/bin/env python3
from adafruit_motorkit import MotorKit
from computer_vision.msg import motor_throttle
import flask
import threading
import rospy
import time

kit = MotorKit()
app = flask.Flask(__name__)

@app.route('/throttle')
def get_throttle():
    kit.motor1.throttle = float(flask.request.args.get('Lthrottle'))
    kit.motor2.throttle = float(flask.request.args.get('Rthrottle'))

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
        threading.Thread(target=lambda: app.run(host="0.0.0.0", port=8080)).start()
        subscriber()
    except rospy.ROSInterruptException:
        rospy.loginfo('Motor controller node terminated.')
