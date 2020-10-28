import time
import sys
import signal
import serial

from std_msgs.msg import Bool
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class ArduinoSensorCollector(Node):

    def __init__(self):
        super().__init__('arduinosensorcolelctor')

        # topic publisher motoremergencybreak
        # subscribe for emergency breaks
        # breaks must be overruled to allow power on the engines
        self.motorEmergencyBreakPublisher = self.create_publisher(Bool, 'motoremergencybreak', 1)

        # #topic publsiher frontdistance
        # Front distance
        # - distance
        self.frontDistancePublisher = self.create_publisher(Int16, 'frontdistance', 1)

        # #topic publisher scannerdistance
        # scanner distance
        # - degree
        # - distance
        self.distancePublisher = self.create_publisher(Int16MultiArray, 'scannerdistance', 1)

        # Setup serial connection
        self.arduino = serial.Serial('/dev/ttyUSB0',115200,timeout=.1)


        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer_callback()


    def timer_callback(self):
        frontMsg = Int16()
        bumperMsg = Bool()
        distanceMsg = Int16MultiArray()

        # Message type
        # 0 send bumper status
        # 1 front US
        # 2 front IR
        # 3 scanner US
        # 4 scanner IR
        #
        # Message format
        # type:angle:distance:speed

        while 1:
            #    with serial.Serial('/dev/ttyUSB0',115200,timeout=1) as ser:
            #line = self.arduino.readline()[:-2]
            sensorLine = self.arduino.readline()[:]
            if sensorLine:
                sensorLine = sensorLine.decode("utf-8")
                sensorMessageArray = sensorLine.split(':')
                if sensorMessageArray[0] == 0: # Front bumper
                    bumperMsg.data = sensorMessageArray[2]
                    self.motorEmergencyBreakPublisher.publish(bumperMsg)
                elif sensorMessageArray[0] < 3: # Front sensor
                    frontMsg.data = sensorMessageArray[2]
                    self.frontDistancePublisher.publish(frontMsg)
                else: # Scanner
                    del sensorMessageArray[0]
                    distanceMsg.data = sensorMessageArray
                    self.distancePublisher.publish(distanceMsg)


def main(args=None):
    rclpy.init(args=args)

    sensorcollector = ArduinoSensorCollector()

    rclpy.spin(sensorcollector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensorcollector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()