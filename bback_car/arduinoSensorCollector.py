import signal
from std_msgs.msg import Bool
from std_msgs.msg import Int16MultiArray
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
import serial


def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class ArduinoSensorCollector(Node):

    def __init__(self):
        super().__init__('arduinosensorcolelctor')

        # topic publisher motorstatus 
        # Int16 array
        # - speed, -1 unknown, m/s based on motor rotation
        # - emergencybreak, is emergency break pulled or not
        # - left power, -100 to 100, negativ backwards
        # - right power, -100 to 100, negativ backwards
        
        self.motorEmergencyBreakPublisher = self.create_publisher(Bool, 'motoremergencybreak', 1)


arduino = serial.Serial('/dev/ttyUSB0',115200,timeout=.1)
while 1:
#    with serial.Serial('/dev/ttyUSB0',115200,timeout=1) as ser:
        line = arduino.readline()[:-2]
        if line:
            line = line.decode("utf-8") 
            print(line)
