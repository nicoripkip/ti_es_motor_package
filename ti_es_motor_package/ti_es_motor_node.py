import rclpy
from rclpy.node import Node
import time
from dataclasses import dataclass
from enum import Enum
from std_msgs.msg import String
import RPi.GPIO as GPIO
import json


DEFAULT_MOTOR_FREQ  = 1000
GPIO_HIGH           = 1
GPIO_LOW            = 0


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
        self.declare_parameter("motor_name", "")
        self.declare_parameter("motor_type", 0)
        self.declare_parameter("step_pin", 0)
        self.declare_parameter("direction_pin", 0)
        self.declare_parameter("sleep_pin", 0)

        # Get parameters
        self.motor_name         = self.get_parameter("motor_name").value
        self.motor_type         = self.get_parameter("motor_type").value
        self.step_pin           = self.get_parameter("step_pin").value
        self.direction_pin      = self.get_parameter("direction_pin").value
        self.sleep_pin          = self.get_parameter("sleep_pin").value

        # Setup pins for controlling the motor driver
        GPIO.setmode(GPIO.BCM)

        self.motor_data = None

        # Configure the pins
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.sleep_pin, GPIO.IN)

        # Setup subscribers
        self.motor_subscriber   = self.create_subscription(String, "/ti/es/motor_data", self.motor_data_callback, 10) 

        # Setup publishers
        self.log_publisher      = self.create_publisher(String, "/ti/es/logger_data", 10)

        # Setup timers
        self.timer              = self.create_timer(0.0001, self.control_loop_callback)


    # This method controls the logic of the motor  
    def control_loop_callback(self):
        if self.motor_controller_shutdown():
            print("Motor has shutdown!")

        if self.motor_data != None:
            
            # Make sure that the direction is set each loop
            GPIO.output(self.direction_pin, GPIO_LOW)

            self.motor_step()

        else:
            print("Motor does not have any instructions!")


    # This method processes the control data for the motors
    def motor_data_callback(self, msg):
        print(msg.data)

        data = json.loads(msg.data)
        direc = None

        if self.motor_type == data["type"]:
            print("Data for the correct motor!")

            match data["direction"]:
                case 0x01:
                    direc = MotorDirection.MOTOR_DIRECTION_LEFT
                case 0x02:
                    direc = MotorDirection.MOTOR_DIRECTION_RIGHT

            self.motor_data = MotorData(data["type"], direc, data["step"])


    # Method to detect if the motor controller has failed or not
    def motor_controller_shutdown(self):
        return False


    def motor_step(self):
         # turn the motor 1 step

        print("Performing a step")

        GPIO.output(self.step_pin, GPIO_HIGH)
        time.sleep(0.000001)
        GPIO.output(self.step_pin, GPIO_LOW)
        time.sleep(0.000001)


# Main function to run the package
def main(args=None):
    rclpy.init(args=args)

    mn = MotorNode()

    print('Hi from ti_es_motor_package.')

    rclpy.spin(mn)


if __name__ == '__main__':
    main()
