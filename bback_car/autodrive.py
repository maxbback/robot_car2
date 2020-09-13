import rclpy
from rclpy.node import Node
import RPi.GPIO as gpio
import time
import sys
import signal
from std_msgs.msg import Int32
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType


class AutoDrive(Node):

    def __init__(self):
        super().__init__('autodrive')
        self.subscription = self.create_subscription(
            Int32,
            'direction',
            self.listener_callback,1)
#           self.listener_callback,rmw_qos_profile_sensor_data)
#https://github.com/ros2/rmw/blob/8ea66dbbe89e78318cd2f2b4e7d8da51211d67bb/rmw/include/rmw/qos_profiles.h
        self.subscription  # prevent unused variable warning

        self.speed_subscription = self.create_subscription(
            Int32,
            'distance',
            self.speed_listener_callback,1)
#           self.listener_callback,rmw_qos_profile_sensor_data)
#https://github.com/ros2/rmw/blob/8ea66dbbe89e78318cd2f2b4e7d8da51211d67bb/rmw/include/rmw/qos_profiles.h
        self.speed_subscription  # prevent unused variable warning

        #define parameeters
        self.declare_parameter("speed")
        my_new_param = rclpy.parameter.Parameter(
            "speed",
            rclpy.Parameter.Type.INTEGER,
            30
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

        self.speed = self.get_parameter("speed").get_parameter_value().integer_value

        self.get_logger().info('Speed limit: "%d"' % self.speed)

        self.pinMotorAForwards = 9
        self.pinMotorABackwards = 10
        self.pinMotorBForwards = 7
        self.pinMotorBBackwards = 8
        self.Frequency = 20
        # How long the pin stays on each cycle, as a percent
        self.DutyCycleA = 30
        self.DutyCycleB = 30
        # Setting the duty cycle to 0 means the motors will not turn
        self.Stop = 0
        gpio.setmode(gpio.BCM)
        gpio.setwarnings(False)
        gpio.setup(self.pinMotorAForwards, gpio.OUT)
        gpio.setup(self.pinMotorABackwards, gpio.OUT)
        gpio.setup(self.pinMotorBForwards, gpio.OUT)
        gpio.setup(self.pinMotorBBackwards, gpio.OUT)
        #Distance Variables
        self.hownear = 15.0
        self.reversetime = 0.5
        self.turntime = 0.75

        # Set the GPIO to software PWM at 'Frequency' Hertz
        self.pwmMotorAForwards = gpio.PWM(self.pinMotorAForwards, self.Frequency)
        self.pwmMotorABackwards = gpio.PWM(self.pinMotorABackwards, self.Frequency)
        self.pwmMotorBForwards = gpio.PWM(self.pinMotorBForwards, self.Frequency)
        self.pwmMotorBBackwards = gpio.PWM(self.pinMotorBBackwards, self.Frequency)

        # Start the software PWM with a duty cycle of 0 (i.e. not moving)
        self.pwmMotorAForwards.start(self.Stop)
        self.pwmMotorABackwards.start(self.Stop)
        self.pwmMotorBForwards.start(self.Stop)
        self.pwmMotorBBackwards.start(self.Stop)

    # Turn all motors off
    def stopmotors(self):
        self.pwmMotorAForwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorABackwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorBForwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.Stop)


    # Turn both motors forwards
    def forwards(self):
        self.pwmMotorAForwards.ChangeDutyCycle(self.DutyCycleA)
        self.pwmMotorABackwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorBForwards.ChangeDutyCycle(self.DutyCycleB)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.Stop)


    # Turn both motors backwards
    def backwards(self):
        self.pwmMotorAForwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorABackwards.ChangeDutyCycle(self.DutyCycleA)
        self.pwmMotorBForwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.DutyCycleB)


    # Turn left
    def left(self):
        self.pwmMotorAForwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorABackwards.ChangeDutyCycle(self.DutyCycleA)
        self.pwmMotorBForwards.ChangeDutyCycle(self.DutyCycleB)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.Stop)


    # Turn Right
    def right(self):
        self.pwmMotorAForwards.ChangeDutyCycle(self.DutyCycleA)
        self.pwmMotorABackwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorBForwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.DutyCycleB)



    # Move back a little, then turn right
    def avoidobstacle(self):
        # Back off a little
        print("Backwards")
        self.backwards()
        time.sleep(self.reversetime)
        self.stopmotors()

        # Turn right
        print("Right")
        self.right()
        time.sleep(self.turntime)
        self.stopmotors()
        time.sleep(self.turntime)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%d"' % msg.data)
        if msg.data < 10:
            self.stopDirection = 0
            self.leftDirection = 1
            self.rightDirection = 2
            self.forwardDirection = 3
            self.backwardDirection = 4
            if msg.data == self.stopDirection:
                self.stopmotors()
            elif msg.data == self.leftDirection:
                self.left()
            elif msg.data == self.rightDirection:
                self.right()
            elif msg.data == self.forwardDirection:
                self.forwards()
            elif msg.data == self.backwardDirection:
                self.backwards()
#            time.sleep(1)
#            self.stopmotors()

    def speed_listener_callback(self,msg):
        self.get_logger().info('I heard distance: "%d"' % msg.data)
        if msg.data > 100:
            self.DutyCycleA = 40
            self.DutyCycleB = 40
        elif msg.data > 30:
            self.DutyCycleA = 30
            self.DutyCycleB = 30
        else:
            self.DutyCycleA = 20
            self.DutyCycleB = 20

def main(args=None):
    rclpy.init(args=args)

    autodrive0 = AutoDrive()

    rclpy.spin(autodrive)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    autodrive.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
