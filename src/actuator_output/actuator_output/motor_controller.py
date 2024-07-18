import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from actuator_output.lib.PiMotor import PiMotor

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Float64,
            'pid_output',
            self.set_motor_speed,
            2)
        self.subscription  # prevent unused variable warning

        self.left_motor = PiMotor.Motor("MOTOR1", 1)
        self.right_motor = PiMotor.Motor("MOTOR2", 1)
        self.std_speed = 40

    def set_motor_speed(self, msg):
        pid_output = msg.data

        # msg.data is positive if the line is to the right of the robot, negative if to the left
        if msg.data >= 0:
            left_motor_throttle = self.std_speed + pid_output
            right_motor_throttle = self.std_speed - pid_output
        else:
            left_motor_throttle = self.std_speed - pid_output
            right_motor_throttle = self.std_speed + pid_output
        
        self.left_motor.forward(self.clamp(left_motor_throttle, 0, 100))
        self.right_motor.forward(self.clamp(right_motor_throttle, 0, 100))

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))
        


def main(args=None):
    rclpy.init(args=args)

    pid_subscriber = MotorController()

    rclpy.spin(pid_subscriber)

    pid_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




