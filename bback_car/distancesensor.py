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

class distanceSensor(Node):

    def __init__(self):
        super().__init__('distancesensor')
        self.frontpublisher = self.create_publisher(Int32, 'frontdistance', 1)
        self.leftpublisher = self.create_publisher(Int32, 'leftdistance', 1)
        self.rightpublisher = self.create_publisher(Int32, 'rightdistance', 1)
        self.distancepublisher = self.create_publisher(Int32, 'distance', 1)
        self.directionpublisher = self.create_publisher(Int32, 'direction', 1)

        self.leftdistance = 100
        self.frontdistance = 100
        self.rightdistance = 100
        self.distance = 100

        GPIO.setmode(GPIO.BCM)
        self.frontTrig = 17 # 7th
        self.frontEcho = 18 # 6th
        self.leftTrig = 22 # 7th
        self.leftEcho = 23 # 6th
        self.rightTrig = 24 # 7th
        self.rightEcho = 25 # 6th

        self.stopDirection = 0
        self.leftDirection = 1
        self.rightDirection = 2
        self.forwardDirection = 3
        self.backwardDirection = 4

        GPIO.setup(self.frontTrig, GPIO.OUT)
        GPIO.setup(self.frontEcho, GPIO.IN)
        GPIO.setup(self.leftTrig, GPIO.OUT)
        GPIO.setup(self.leftEcho, GPIO.IN)
        GPIO.setup(self.rightTrig, GPIO.OUT)
        GPIO.setup(self.rightEcho, GPIO.IN)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def getdistance(self,sensorid):

        rpinTrigger = 24
        rpinEcho = 25
        lpinTrigger = 22
        lpinEcho = 23
        fpinTrigger = 17
        fpinEcho = 18
        distance = 0



        # Set pins as output and input
        GPIO.setup(fpinTrigger, GPIO.OUT)  # Trigger
        GPIO.setup(fpinEcho, GPIO.IN)  # Echo
        GPIO.setup(lpinTrigger, GPIO.OUT)  # Trigger
        GPIO.setup(lpinEcho, GPIO.IN)  # Echo
        GPIO.setup(rpinTrigger, GPIO.OUT)  # Trigger
        GPIO.setup(rpinEcho, GPIO.IN)  # Echo


        if sensorid == 0:
            #self.get_logger().info('Front distance')
            pinTrigger = fpinTrigger
            pinEcho = fpinEcho
            sensorid = 1
        elif sensorid == 1:
            #self.get_logger().info('Left distance')
            pinTrigger = lpinTrigger
            pinEcho = lpinEcho
            sensorid = 2
        elif sensorid == 2:
            #self.get_logger().info('Right distance')
            pinTrigger = rpinTrigger
            pinEcho = rpinEcho
            sensorid = 0

        # Set trigger to GPIO.LOW (Low)
        GPIO.output(pinTrigger, GPIO.LOW)

        # Allow module to settle
        time.sleep(0.0001)

        # Send 10us pulse to trigger
        GPIO.output(pinTrigger, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(pinTrigger, GPIO.LOW)

        # Start the timer
        StartTime = time.time()

        # The start time is reset until the Echo pin is taken high (==1)
        while GPIO.input(pinEcho) == 0:
            StartTime = time.time()

        StopTime = StartTime
        # Stop when the Echo pin is no longer high - the end time
        while GPIO.input(pinEcho) == 1:
            StopTime = time.time()
            # If the sensor is too close to an object, the Pi cannot
            # see the echo quickly enough, so we have to detect that
            # problem and say what has happened.
            ElapsedTime = StopTime - StartTime
            if ElapsedTime >= 0.04:
                self.get_logger().info('To close to see: "%.1f" ' % ElapsedTime)
                StopTime = StartTime
                break

        # Calculate pulse length
        ElapsedTime = StopTime - StartTime

        # distance pulse travelled in that time is
        # time multiplied by the speed of sound (cm/s)
        distance = ElapsedTime * 34326

        # That was the distance there and back so halve the value
        distance = round(distance / 2)

        #self.get_logger().info('distance: "%d"' % distance)

        #time.sleep(0.5)
        return distance
    def sendDirection(self):
        msg = Int32()
        self.direction = self.forwardDirection # go backward
        if 15 > self.frontdistance:
            self.direction = self.rightDirection # go backward
        if 15 > self.leftdistance:
            self.direction = self.rightDirection # go right
        if 15 > self.rightdistance:
            self.direction = self.leftDirection # go left

        self.distance = 1000
        if self.distance > self.frontdistance:
            self.distance = self.frontdistance
        if self.distance > self.leftdistance:
            self.distance = self.leftdistance
        if self.distance > self.rightdistance:
            self.distance = self.rightdistance
        self.get_logger().info('Direction: %d :f %d l %d r %d' %(self.direction,self.frontdistance,self.leftdistance,self.rightdistance))
        self.get_logger().info('Direction: "%d"' % self.direction)
        msg.data = self.distance
        self.distancepublisher.publish(msg)
        msg.data = self.direction
        self.directionpublisher.publish(msg)

    def timer_callback(self):
        msg = Int32()
        self.frontdistance = self.getdistance(0)
        msg.data = self.frontdistance
        self.frontpublisher.publish(msg)
        self.leftdistance = self.getdistance(1)
        self.sendDirection()
        msg.data = self.leftdistance
        self.leftpublisher.publish(msg)
        self.rightdistance = self.getdistance(2)
        msg.data = self.rightdistance
        self.rightpublisher.publish(msg)



def main(args=None):
#    try:
    rclpy.init(args=args)

    distancesensor = distanceSensor()

    rclpy.spin(distancesensor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    distancesensor.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()
#    except KeyboardInterrupt:
        # Reset GPIO settings
#        GPIO.cleanup()

if __name__ == '__main__':
    main()
