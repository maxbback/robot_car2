# IR bumper 
import rclpy
from rclpy.node import Node


import RPi.GPIO as GPIO
import time
import sys
import signal
from std_msgs.msg import Bool


def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        GPIO.cleanup()
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class IRbumper(Node):

    def __init__(self):
        super().__init__('irbumper')
        self.bumper_publisher = self.create_publisher(Bool, 'motoremergencybreak', 1)

        self.irPin = 27 # Bumper port

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.irPin, GPIO.IN)

        self.msg = Bool()

        # only send messages if something changed
        self.lastState = -1


        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        # Stop when the Echo pin is no longer high - the end time
        self.msg.data = GPIO.input(irPin)
        if self.lastState != msg.data:
            self.lastState = msg.data
            self.bumper_publisher.publish(msg)


def main(args=None):
#    try:
    rclpy.init(args=args)

    irbumper = IRbumper()

    rclpy.spin(irbumper)

    irbumper.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
