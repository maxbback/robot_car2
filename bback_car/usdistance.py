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
from std_msgs.msg import Int32MultiArray


def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        GPIO.cleanup()
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class USdistance(Node):

    def __init__(self):
        super().__init__('usdistance')

        self.firstIteration = 1 # If this is 1 then we force an initiation of distance history
        self.maxHistory = 10

        self.pollIterator = -1

        self.publisher = self.create_publisher(Int32MultiArray, 'usdistance', 1)

        self.stopDirection = 0
        self.leftDirection = 1
        self.rightDirection = 2
        self.forwardDirection = 3
        self.backwardDirection = 4

        self.maxDistance = 400

        self.distance = self.maxDistance
        self.direction = self.forwardDirection
        self.directionLast = -1

        # keep last collection points use them for estimation collision risk
        self.distanceArray = [
            [400,400,400,400,400,400,400,400,400,400],
            [400,400,400,400,400,400,400,400,400,400],
            [400,400,400,400,400,400,400,400,400,400]
        ]


        # Zone model
        # 1000 Front (center)
        # 1100 Left front to center
        # 1200 Right front to center
        # 2100 Left side Front to center
        # 2000 Left (enter)
        # 2200 Left side Back to center
        # 3100 Right side Front
        # 3000 Right (enter)
        # 3200 Right side Back to center
        # 4000 Back (center)
        # 4100 Left back to center
        # 4200 Right back

        self.sensorList = []
        self.sensorList.append(0)
        # sensor zone,trigge,echo
        self.sensorArray = [[1000,24,25],[2100,22,23],[3100,18,27]] #trigger, echo
        self.nrSensors = 0
        for i in self.sensorArray:
            self.nrSensors += 1
            self.sensorList.append(i[0])
        self.sensorList[0] = self.nrSensors #Number of sensors

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(GPIO.LOW)


#        GPIO.setup(self.frontTrig, GPIO.OUT)
#        GPIO.setup(self.frontEcho, GPIO.IN)
#        GPIO.setup(self.leftTrig, GPIO.OUT)
#        GPIO.setup(self.leftEcho, GPIO.IN)
#        GPIO.setup(self.rightTrig, GPIO.OUT)
#        GPIO.setup(self.rightEcho, GPIO.IN)
        for sensorX in range(self.nrSensors):
            # Set pins as output and input
            GPIO.setup(self.sensorArray[sensorX][1], GPIO.OUT)  # Trigger
            GPIO.output(self.sensorArray[sensorX][1], GPIO.LOW)
            GPIO.setup(self.sensorArray[sensorX][2], GPIO.IN)  # Echo
        time.sleep(0.1)


        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def getDistance(self,sensorId):


        # Set trigger to GPIO.LOW (Low)
        GPIO.output(self.sensorArray[sensorId][1], GPIO.LOW)
        self.get_logger().info('Set triger to low %d ' % sensorId)

        # Allow module to settle
        time.sleep(2.001)

        # Send 10us pulse to trigger
        GPIO.output(self.sensorArray[sensorId][1], GPIO.HIGH)
        time.sleep(1.00001)
        GPIO.output(self.sensorArray[sensorId][1], GPIO.LOW)

        # Start the timer
        StartTime = time.time()
        self.get_logger().info('Sent echo %d ' % sensorId)

        # The start time is reset until the Echo pin is taken high (==1)
        while GPIO.input(self.sensorArray[sensorId][2]) == 0:
            StartTime = time.time()

        self.get_logger().info('Sensor high ')
        StopTime = StartTime
        # Stop when the Echo pin is no longer high - the end time
        while GPIO.input(self.sensorArray[sensorId][2]) == 1:
            StopTime = time.time()
            # see the echo quickly enough, so we have to detect that
            # problem and say what has happened.
            ElapsedTime = StopTime - StartTime
            if ElapsedTime >= 0.04:
                self.get_logger().info('To close to see: "%d %d %.1f" ' %(sensorId,self.sensorArray[sensorId][2],ElapsedTime))
                StopTime = StartTime
                break

        # Calculate pulse length
        ElapsedTime = StopTime - StartTime

        # distance pulse travelled in that time is
        # time multiplied by the speed of sound (cm/s)
        distance = ElapsedTime * 34326

        # That was the distance there and back so halve the value
        distance = round(distance / 2)
        self.get_logger().info('distiance: "%d %d" ' %(sensorId,distance))

        #self.get_logger().info('distance: "%d"' % distance)

        #time.sleep(0.5)
        return distance


    def calcSpeed(self,distance):
        # based on how fast we move to an abstacle the more we reduce speed
        # we calculate this for each direction
        if distance < 100:
            return distance
        else:
            return 100

    def calcCollisionRisk(self,sensorId):
        iPrevious = self.pollIterator
        currentDistance =  self.distanceArray[sensorId][self.pollIterator]
        if currentDistance > 50:
            return 0

        riskArray = [0,0,0]
        for x in range(3): # Check last 2 measure ments
            iPrevious -= 1
            if iPrevious == -1:
                iPrevious = 9

            distanceChange = self.distanceArray[sensorId][iPrevious] - currentDistance
            if distanceChange > 0: # We are getting closer
                if distanceChange > 100: #Something is not corect
                    riskArray[x] = 0
                else:
                    if currentDistance > 0:
                        riskArray[x] = round(distanceChange/currentDistance*100/(x+1))
                    else:
                        riskArray[x] = 0
                        # we do not have a correct measurment
                    # d = 100
                    # c = 10
                    # r = c / d = 0.1
                    #
                    # d = 30
                    # c = 10
                    # r = c / d = 0.33
        #for x in range(2): # Check the trend, if trend is increasing risk is higher
        # find a good formula

        return riskArray[0]

    def timer_callback(self):
        msg = Int32MultiArray()
        self.pollIterator += 1
        if self.pollIterator == 10:
            self.pollIterator = 0

        distanceArray = []
        speedArray = []
        collisionRiskArray = []

        for sensorX in range(self.nrSensors):
            # Get distance
            distance = self.getDistance(sensorX)
            distanceArray.append(distance)
            # Save distance in global array for iteration compariation
            # used for calculating how fast we move to an obstacle
            if self.firstIteration == 1:
                for i in range(self.maxHistory):
                    self.distanceArray[sensorX][i] = distance
            else:
                self.distanceArray[sensorX][self.pollIterator] = distance
            # calculate speed recomendation based on distance
            speedArray.append(self.calcSpeed(distance))
            collisionRiskArray.append(self.calcCollisionRisk(sensorX))

        # MArk initiation done
        self.firstIteration = 0
        #array = []
        #my_array_for_publishing = Int32MultiArray(data=array)
        #pub = node.create_publisher(Int32MultiArray, 'Chatter')
        #a=[1,2,3,4,5]
        #myarray = Int32MultiArray(data=a)
        #pub.publish(myarray)

        msgArray = []
        msgArray = self.sensorList + distanceArray + speedArray + collisionRiskArray

        msg.data = msgArray
        self.publisher.publish(msg)


def main(args=None):
#    try:
    rclpy.init(args=args)

    usdistance = USdistance()

    rclpy.spin(usdistance)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    usdistance.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()
#    except KeyboardInterrupt:
        # Reset GPIO settings
#        GPIO.cleanup()

if __name__ == '__main__':
    main()
