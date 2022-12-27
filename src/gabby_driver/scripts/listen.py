#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy

from gabby_msgs import GetOdometry

left_rotation = 0
right_rotation = 0

# Checks if a wheel is moving forward or backward
def inFowardMode(pin):
    if pin == 7:
        # check if pin 26 is HIGH and pin 24 is LOW
        if GPIO.input(26) == GPIO.HIGH and GPIO.input(24) == GPIO.LOW:
            return True
        else:
            return False
    else:
        if GPIO.input(13) == GPIO.HIGH and GPIO.input(15) == GPIO.LOW:
            return True
        else:
            return False    

# Triggered by an interrupt, counts the increments of the encoder 
# (20 increments per revolution)
def eventCallback(pin):
    global left_rotation, right_rotation

    if pin == 7:
        if inFowardMode(pin):
            right_rotation += 1/20
        else:
            right_rotation -= 1/20
    else: 
        if inFowardMode(pin):
            left_rotation += 1/20
        else:
            left_rotation -= 1/20

def setup():
    # Set up the GPIO pins
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(7, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Have an interrupt when the value of the pin changes from 0 to 1 or the reverse
    GPIO.add_event_detect(7, GPIO.BOTH, callback=eventCallback, bouncetime=25)
    GPIO.add_event_detect(12, GPIO.BOTH, callback=eventCallback, bounctime=25)

# Service callback, returns the current rotation of the wheels
def handle_odometry_request(req):
    global left_rotation, right_rotation
    return GetOdometry(left_rotation, right_rotation)

def server():
    rospy.init_node("odometry_server")
    service = rospy.Service("get_odometry", GetOdometry, handle_odometry_request)

    rospy.spin()

if __name__ == "__main__":
    setup()
    server()