import rclpy
from rclpy.node import Node
import time
from dataclasses import dataclass
from enum import Enum
from std_msgs.msg import String
import pigpio


DEFAULT_MOTOR_FREQ = 1000


# This enum contains the direction to which the motor can turn
class MotorDirection(Enum):
    MOTOR_DIRECTION_LEFT    = 0x01
    MOTOR_DIRECTION_RIGHT   = 0x02


# Parameters for control motor
@dataclass
class MotorData:
    type: int
    direction: MotorDirection
    step_freq: int


class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_node")

        # Setup parameters
        self.declare_parameter("motor_type", 0)
        self.declare_parameter("step_pin", 0)
        self.declare_parameter("direction_pin", 0)

        # Get parameters
        self.motor_type         = self.get_parameter("motor_type").value
        self.step_pin           = self.get_parameter("step_pin").value
        self.direction_pin      = self.get_parameter("direction_pin").value

        # Setup pins for controlling the motor driver
        self.gpio = pigpio.pi()
        if not self.gpio.connected:
            print("Can´t connect to gpio deamon!")

        self.motor_data = None

        # Set the step pin als PWM
        # self.step_pwm = GPIO.PWM(self.step_pin, DEFAULT_MOTOR_FREQ)

        # Setup subscribers
        self.motor_subscriber   = self.create_subscription(String, "ti/es/motor_data", self.motor_data_callback, 10) 

        # Setup publishers
        self.log_publisher      = self.create_publisher(String, "ti/es/logger_data", 10)

        # Setup timers
        self.timer              = self.create_timer(1, self.control_loop_callback)


    # This method controls the logic of the motor  
    def control_loop_callback(self):
        if self.motor_data != None:
            pass
        else:
            print("Motor does not have any instructions!")


    # This method processes the control data for the motors
    def motor_data_callback(self, msg):
        data = json.loads(msg.data)
        direc = None

        match data["direction"]:
            case 0x01:
                direc = MotorDirection.MOTOR_DIRECTION_LEFT
            case 0x02:
                direc = MotorDirection.MOTOR_DIRECTION_RIGHT

        self.motor_data = MotorData(data["type"], direc, data["step"])
        

def main(args=None):
    rclpy.init(args=args)

    mn = MotorNode()

    print('Hi from ti_es_motor_package.')

    rclpy.spin(mn)


if __name__ == '__main__':
    main()
