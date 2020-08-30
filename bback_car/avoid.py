import rclpy
from rclpy.node import Node
import RPi.GPIO as gpio
import time
import sys
import signal
from std_msgs.msg import Int32


class Avoid(Node):

    def __init__(self):
        super().__init__('avoid')
        self.subscription = self.create_subscription(
            Int32,
            'frontdistance',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
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
        if msg.data < self.hownear:
            self.stopmotors()
            self.avoidobstacle()
            self.forwards()
        else:
            self.forwards()



def main(args=None):
    rclpy.init(args=args)

    avoid = Avoid()

    rclpy.spin(avoid)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avoid.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
