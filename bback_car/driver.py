import rclpy
from rclpy.node import Node
import RPi.GPIO as gpio
import time
import sys
import signal
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

# Auto drive design
# Speed is related to obstacles in the path, slow down if we will hit
# Front is clear go forward
# if left or right is closer than X keep track on if we keep closer
# Adjust so we do not get closer than y
#

class Driver(Node):

    def __init__(self):
        super().__init__('driver')
        # IR Bumper, we are to close
        self.subscription_bumper = self.create_subscription(
            Int32,
            'frontbumper',
            self.bumper_listener_callback,1)

        self.subscription_bumper  # prevent unused variable warning

        self.subscription_usdistance = self.create_subscription(
            Int32MultiArray,
            'usdistance',
            self.usdistance_listener_callback,1)
#           self.listener_callback,rmw_qos_profile_sensor_data)
#https://github.com/ros2/rmw/blob/8ea66dbbe89e78318cd2f2b4e7d8da51211d67bb/rmw/include/rmw/qos_profiles.h
        self.subscription_usdistance  # prevent unused variable warning

        #msgArray = self.sensorList + distanceArray + speedArray + collisionRiskArray
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

        # Right motor
        self.pinMotorAForwards = 10
        self.pinMotorABackwards = 9
        # Left motor
        self.pinMotorBForwards = 8
        self.pinMotorBBackwards = 7
        self.Frequency = 20
        # How long the pin stays on each cycle, as a percent
        self.MinCycle = 15
        self.DutyCycleA = 30
        self.DutyCycleB = 30
        self.DutyCycleACompensate = 0 # compensation from inbalanced drift
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


        self.objectiveDirection = 0


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
        self.get_logger().info('Stop ')
        self.pwmMotorAForwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorABackwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorBForwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.Stop)


    # Turn both motors forwards
    def forwards(self):
        self.get_logger().info('Forward ')
        self.pwmMotorAForwards.ChangeDutyCycle(self.DutyCycleA)
        self.pwmMotorABackwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorBForwards.ChangeDutyCycle(self.DutyCycleB)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.Stop)


    # Turn both motors backwards
    def backwards(self,):
        self.get_logger().info('Backward ')
        self.pwmMotorAForwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorBForwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorABackwards.ChangeDutyCycle(self.DutyCycleA)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.DutyCycleB)


    # Turn left
    def left(self):
        self.get_logger().info('Left ')
        self.pwmMotorAForwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorABackwards.ChangeDutyCycle(self.DutyCycleA)
        self.pwmMotorBForwards.ChangeDutyCycle(self.DutyCycleB)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.Stop)

    def turnLeft(self,collisionRisk):
        turnTime = self.turntime * collisionRisk / 100 + 0.1
        self.get_logger().info('Turn left : %d %d' %(collisionRisk,turnTime))
        self.setSpeed(50)
        self.left()
        time.sleep(self.turntime)
        self.stopmotors()
#        self.forwards()

    # Turn Right
    def right(self):
        self.get_logger().info('Right ')
        self.pwmMotorAForwards.ChangeDutyCycle(self.DutyCycleA)
        self.pwmMotorABackwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorBForwards.ChangeDutyCycle(self.Stop)
        self.pwmMotorBBackwards.ChangeDutyCycle(self.DutyCycleB)

    def turnRight(self,collisionRisk):
        turnTime = self.turntime * collisionRisk / 100 + 0.1
        self.get_logger().info('Turn Right : %d %d' %(collisionRisk,turnTime))
        self.setSpeed(50)
        self.right()
        time.sleep(self.turntime)
        self.stopmotors()
#        self.forwards()


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

    def setSpeed(self,speed):
        # The lower speed the shorter engine time
        newSpeed = round(30*speed/100) + 10
        newDriveTime = round(10*speed/100) + 10
        self.get_logger().info('Speed : %d %d' %(newSpeed,newDriveTime))

        self.DutyCycleA = newSpeed + self.DutyCycleACompensate
        self.DutyCycleB = newSpeed
#        self.Frequency = newDriveTime


    def bumper_listener_callback(self,msg):
        self.get_logger().info('Bumper: %d' % msg.data)
        # full break
        if msg.data == 1:
            self.stopmotors()

    def usdistance_listener_callback(self,msg):
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

        msgData = ': '.join(map(str, msg.data))
        self.get_logger().info('Msg : %s' % msgData)

        #msgArray = self.sensorList + distanceArray + speedArray + collisionRiskArray
        nrSensors = msg.data[0]
        sensorList = []
        distanceArray = []
        speedArray = []
        collisionRiskArray = []
        x = 1
        for i in range(nrSensors):
            sensorList.append(msg.data[i+1])
            distanceArray.append(msg.data[i+1+nrSensors])
            speedArray.append(msg.data[i+1+nrSensors + nrSensors])
            collisionRiskArray.append( msg.data[i+1+nrSensors + nrSensors + nrSensors])

        self.get_logger().info('d & c : %d %d' %(distanceArray[i],collisionRiskArray[i]))

        # Hardcoded fow now
        # Check if we shall slow down
        proposedSpeed = 100
        collistion = 0
        goForward = 0
        self.get_logger().info('direction : %d %d' %(self.objectiveDirection,distanceArray[0]))
        if distanceArray[0] < 15: # To close to wall, change direction
            if self.objectiveDirection == 1:
                self.turnLeft(collisionRiskArray[1])
            elif self.objectiveDirection == 2:
                self.turnRight(collisionRiskArray[2])
            elif distanceArray[1] > distanceArray[2]: # Turn left
                self.objectiveDirection = 1
                self.turnLeft(collisionRiskArray[1])
            else:
                self.objectiveDirection = 2
                self.turnRight(collisionRiskArray[2])
            return
        else:
            self.objectiveDirection = 0

        for i in range(nrSensors):
            if collisionRiskArray[i] > 0 or  ( i == 0 and distanceArray[i] < 15):
                goForward = 1
                proposedSpeed = speedArray[i]
                self.setSpeed(speedArray[i])
                if i == 0: # Front shal we turn left or right
                    if distanceArray[i] < 10:
                        self.setSpeed(20)
                        self.get_logger().info('Front to close revere : %d %d' %(distanceArray[i],collisionRiskArray[i]))
                        self.backwards()
                        time.sleep(self.reversetime)
                        self.stopmotors()

                elif i == 1: # To close to left side
                    self.setSpeed(speedArray[1])
                    self.turnRight(collisionRiskArray[i])
                else:
                    self.setSpeed(speedArray[2])
                    self.turnLeft(collisionRiskArray[i])
                # Wait for new information
                break
            else:
                self.objectiveDirection = 0
        if goForward == 0:
            self.setSpeed(proposedSpeed)
            self.forwards()


        # Check if we shall adjust left distance
        # Check if we shall adjust right distance



def main(args=None):
    rclpy.init(args=args)

    driver = Driver()

    rclpy.spin(driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
