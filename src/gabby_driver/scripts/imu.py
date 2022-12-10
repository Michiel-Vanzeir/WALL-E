import time
import board
import adafruit_bno055
from operator import add, truediv

i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

while True:
    linear_acc = [0, 0, 0]
    for i in range(10):
        linear_acc = list(map(add, linear_acc, sensor._linear_acceleration))
    
    linear_acc = list(map(truediv, linear_acc, [10]*3))

    print("Linear acceleration (m/s^2): {}".format(tuple(linear_acc)))
    
    print()
    time.sleep(0.5)
