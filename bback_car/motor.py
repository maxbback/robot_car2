import rclpy
from rclpy.node import Node
import RPi.GPIO as gpio
import time
import sys
import signal
from std_msgs.msg import Bool
from std_msgs.msg import Int16MultiArray
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

# Motor node
# This node controles the motors
#
# Inbound topics
# motorpower used to controle the power of the engines and direction
# motoremergencybreak used to stop motors
#
# Outbound topic
# motorspeed contains estimated speed based on motor rotation and latest motor power information

class Motor(Node):

    def __init__(self):
        super().__init__('motor')
        # #topic consumer motorpower 
        # Motor direction and power events
        # - reset emergency break
        # - duration , specifies time motor run before turning them of, 0 do not turn motors of
        # - left power
        # - right power
        # power is from -100 to 100, negativ is backwards
        self.subscription_motorpower = self.create_subscription(
            Int16MultiArray,
            'motorpower',
            self.motorpower_listener_callback,1)

        self.subscription_motorpower  # prevent unused variable warning
        
        # topic consumer motoremergencybreak
        # subscribe for emergency breaks
        # breaks must be overruled to allow power on the engines
        self.subscription_emergencybreak = self.create_subscription(
            Bool,
            'motoremergencybreak',
            self.motor_listener_emergencybreak,1)
        

        self.subscription_emergencybreak  # prevent unused variable warning

        # topic publisher motorstatus 
        # Int16 array
        # - speed, -1 unknown, m/s based on motor rotation
        # - emergencybreak, is emergency break pulled or not
        # - left power, -100 to 100, negativ backwards
        # - right power, -100 to 100, negativ backwards
        
        self.motorStatusPublisher = self.create_publisher(Int16MultiArray, 'motorstatus', 1)


        # PWM allows to pulse the power to an engine with a frequence
        # This is a way to simulate analog voltage to the motor
        # The frequency can be rather high like 1000 = 1khz
        # 1000 means power is turned on 1000 times per second
        # Duty cycle is specifying the % of each pule that the power is on
        # The duty cycle then control how much % power is on and by so the speed
        #
        # Speed is based on how fast the motor rotates and is measuered in rotating per mniute RPM
        # and the size of the wheel
        # if motor is 60 rpm and wheel is 5 cm in diameeter one rotation is 5*pi = 5*3.14 = 15.7 cm
        # In this case it moved 15.7 cm per second
        #
        # with higher frequency we get more cirect speed
        

        #
        self.motorStatusMsg = Int16MultiArray()
        self.motorStatusMsgArray = []
        
        # Break flag, prevent any further movement
        self.emergencyBreak = 0

        # Right motor
        self.pinMotorLForwards = 10
        self.pinMotorLBackwards = 9
        # Left motor
        self.pinMotorRForwards = 8
        self.pinMotorRBackwards = 7
        
        # power frequency is simulating analogue power, by pulsing the power
        self.Frequency = 100 # Turn on and of power frequency in hz
        # Min power duration
        self.MinPower = 15
        # power duration compensattion to adjust for motor difrentiations
        self.leftMotorAdjustment = 0 # compensation from inbalanced drift
        self.rightMotorAdjustment = 0 # compensation from inbalanced drift

        # Setting the duty cycle to 0 means the motors will not turn
        self.Stop = 0

        # Setup gpio
        gpio.setmode(gpio.BCM)
        gpio.setwarnings(False) # Ignore warnings
        # Enabling the outpound motor ports
        gpio.setup(self.pinMotorLForwards, gpio.OUT)
        gpio.setup(self.pinMotorLBackwards, gpio.OUT)
        gpio.setup(self.pinMotorRForwards, gpio.OUT)
        gpio.setup(self.pinMotorRBackwards, gpio.OUT)
 
        # Set the GPIO to software PWM at 'Frequency' Hertz
        self.pwmMotorLForwards = gpio.PWM(self.pinMotorLForwards, self.Frequency)
        self.pwmMotorLBackwards = gpio.PWM(self.pinMotorLBackwards, self.Frequency)
        self.pwmMotorRForwards = gpio.PWM(self.pinMotorRForwards, self.Frequency)
        self.pwmMotorRBackwards = gpio.PWM(self.pinMotorRBackwards, self.Frequency)

        # Start the software PWM with a duty cycle of 0 (i.e. not moving)
        self.pwmMotorLForwards.start(self.Stop)
        self.pwmMotorLBackwards.start(self.Stop)
        self.pwmMotorRForwards.start(self.Stop)
        self.pwmMotorRBackwards.start(self.Stop)

    # Turn all motors off
    def stopMotor(self):
        self.get_logger().info('Stop ')

        self.pwmMotorLForwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorLBackwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorRForwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorRBackwards.ChangeDutyCycle(self.Stop)
        # Publish motor status
        self.motorStatusMsgArray = [0,self.emergencyBreak,0, 0]
        self.motorStatusMsg.data = self.motorStatusMsgArray
        self.motorStatusPublisher.publish(self.motorStatusMsg)

    # Turn motor on
    def runMotor(self,releasebreak, duration, leftPower, rightPower):
        self.get_logger().info('run engine ')
 
        if (self.emergencyBreak == 0 or releasebreak == 1):
            self.emergencyBreak = 0
            if (leftPower >= 0):
                if (leftPower != 0 and leftPower < self.MinPower):
                    leftPower = self.MinPower

                self.pwmMotorLForwards.ChangeDutyCycle(leftPower)
                self.pwmMotorLBackwards.ChangeDutyCycle(self.Stop)
            else:
                if (leftPower > -self.MinPower):
                    leftPower = -self.MinPower

                self.pwmMotorRForwards.ChangeDutyCycle(self.Stop)
                self.pwmMotorRBackwards.ChangeDutyCycle(abs(leftPower))
            # Publish motor status
            self.motorStatusMsgArray = [-1,self.emergencyBreak,leftPower, rightPower]
            self.motorStatusMsg.data = self.motorStatusMsgArray
            self.motorStatusPublisher.publish(self.motorStatusMsg)
            
        if (self.emergencyBreak == 1 or duration > 0):
            if (self.emergencyBreak == 0 and duration > 0): # No emergency
                # if motor shall run only for a specified time in mili seconds
                # Stop motor after duration time
                sleepTime = duration/10
                time.sleep(sleepTime)
            # Stop motor
            self.stopMotor()
            
 
    def setSpeed(self,speed):
        # The lower speed the shorter engine time
        newSpeed = round(30*speed/100) + 10
        newDriveTime = round(10*speed/100) + 10
        self.get_logger().info('Speed : %d %d' %(newSpeed,newDriveTime))

        self.DutyCycleA = newSpeed + self.DutyCycleACompensate
        self.DutyCycleB = newSpeed


    def motor_listener_emergencybreak(self,msg):
        if (msg.data == True):
            self.emergencyBreak = 1
            self.stopMotor()
        else:
            self.emergencyBreak = 0
            
            
            
    def motorpower_listener_callback(self,msg):
        # reset emergency break, duration, left power, right power
        self.runMotor(msg.data[0], msg.data[1], msg.data[2], msg.data[3])



def main(args=None):
    rclpy.init(args=args)

    motor = Motor()

    rclpy.spin(motor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
