import RPi.GPIO as gpio

class Motorshield:
    # Motor pins are in the format [PWM, IN1, IN2]
    # The left motor is the first element, right motor is the second element
    motorpins = [[17, 27, 22], [12, 7, 8]]

    def __init__(self, motorpins=motorpins):
        self.motorpins = motorpins

        gpio.setmode(gpio.BCM)
        gpio.setwarnings(False)
        
        # Set up the GPIO pins
        for motor in self.motorpins:
            for pin in motor:
                gpio.setup(pin, gpio.OUT)
        
        self.pwmpins = [gpio.PWM(pin[0], 100) for pin in self.motorpins]
        print(self.pwmpins)
        for pwm in self.pwmpins:
            pwm.start(0)    

        for motor in self.motorpins:
            gpio.output(motor[0], gpio.HIGH)
            gpio.output(motor[1:], gpio.LOW)

    # Set the left motor throttle
    def setLeftThrottle(self, throttle):
        print(throttle)
        if throttle >= 0:
            # Left motor spins forwards
            gpio.output(self.motorpins[0][1], gpio.HIGH)
            gpio.output(self.motorpins[0][2], gpio.LOW)
        else:
            # Left motor spins backwards
            gpio.output(self.motorpins[0][1], gpio.LOW)
            gpio.output(self.motorpins[0][2], gpio.HIGH)
        
        print("Setting left throttle to " + str(abs(throttle)))
        self.pwmpins[0].ChangeDutyCycle(abs(throttle) * 100)
    
    # Set the right motor throttle
    def setRightThrottle(self, throttle):
        print(throttle)
        if throttle >= 0:
            # Right motor spins forwards
            gpio.output(self.motorpins[1][1], gpio.HIGH)
            gpio.output(self.motorpins[1][2], gpio.LOW)
        else:
            # Right motor spins backwards
            gpio.output(self.motorpins[1][1], gpio.LOW)
            gpio.output(self.motorpins[1][2], gpio.HIGH)

        print("Setting right throttle to " + str(abs(throttle)))
        self.pwmpins[1].ChangeDutyCycle(abs(throttle) * 100)

    # Clean up the GPIO pins
    def shutdown(self):
        for pwm in self.pwmpins:
            pwm.stop()
        gpio.cleanup()


        