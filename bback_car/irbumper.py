# Copyright 2020 SmsTid AB.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# IR bumper is using an ir based obstable sensor
# this is detecting if we are closer than 3 cm it sends a signal, boolen no distance measure

import rclpy
from rclpy.node import Node

#from tutorial_interfaces.msg import Num

import RPi.GPIO as GPIO
import time
import sys
import signal
from std_msgs.msg import Int32


def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        GPIO.cleanup()
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class IRbumper(Node):

    def __init__(self):
        super().__init__('irbumper')
        self.frontbumper = self.create_publisher(Int32, 'frontbumper', 1)

        GPIO.setmode(GPIO.BCM)
        self.irPin = 27 # 7th
        self.lastState = -1


        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int32()

        GPIO.setup(self.irPin, GPIO.IN)

        # Stop when the Echo pin is no longer high - the end time
        msg.data = GPIO.input(irPin)
        if self.lastState != msg.data:
            self.lastState = msg.data
            self.frontbumper.publish(msg)


def main(args=None):
#    try:
    rclpy.init(args=args)

    irbumper = IRbumper()

    rclpy.spin(irbumper)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    irbumper.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()
#    except KeyboardInterrupt:
        # Reset GPIO settings
#        GPIO.cleanup()

if __name__ == '__main__':
    main()
