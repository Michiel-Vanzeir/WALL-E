#! /usr/bin/env python3
import time
from adafruit_motorkit import MotorKit


kit = MotorKit()

def subscriber_test():
    # Make robot go straight
    kit.motor1.throttle = 0.4
    kit.motor2.throttle = 0.5





if __name__ == '__main__':
    subscriber_test()

