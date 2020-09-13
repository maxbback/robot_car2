# robot_car2
Moving this to version 2
- Adding ardunio card for sensor data collection
Multi sensor autodriving car

Current key objectives:
- Avoid objects by moving forward and turning around (sensors is only in front and on the sides)
- Variable speed, slow down when closer to objects
status: meet

Next key level of objective:
- Follow the walls
- Avoid getting trapped, one sugestion is to reverse movements if we are stuck

Comming objectives
- attach camera, use it for recognition
- map room


Version 2
Is inproved by moving ultrasonic side sensors more to the front and replacing side sensors with ir sensors (10-80 cm)
With the new ultrasonic coverage in the front we get a better view of if we are risking to hit an obstacle.
Core components:
2 Motors + battery pack for motors
Motor controller board
Raspberry pi
3 HC-SR04 ultrasonci sensors
3 Sharp IR distance sensors
Arduino Nano kompatibelt V3.0 ATMEGA328 
You also need a bunch of cabels

With the arduino providing analogical input for IR sensors, and I also moved my ultrasonic sensors to arduino.
My sendor prints the result in a : separated format to be onsumed by ros node
Sensor code has been improved to handle some of the non connsisent values from ultrasonic sensors, sometime they generate a high value if it can not get a corrct measuare and that is ignored and previous distance is used.
See ardunio directory.
For IR sensors I used a library for SHARP ir sensors to calculate the right distance based on a complex response curve.
https://drive.google.com/file/d/1yglX0lYazxgC3TkjZ83E_IVw0gYlycW0/view

[![Watch the video](image_car2/IMG_2060.jpeg)](image_car2/IMG_2060.jpeg)
[![Watch the video](image_car2/IMG_2061.jpeg)](image_car2/IMG_2061.jpeg)
[![Watch the video](image_car2/IMG_2062.jpeg)](image_car2/IMG_2062.jpeg)
[![Watch the video](image_car2/IMG_2063.jpeg)](image_car2/IMG_2063.jpeg)

Version 1
Core components:
2 Motors + battery pack for motors
Motor controller board
Raspberry pi
3 HC-SR04 ultrasonci sensors
You also need a bunch of cabels
Current improvement areas is
- When direction is not forward use distance to optimize where to go, try turn and if forward distance is ok go forward a litle
- But in general a  smarter algorithm for movement is needed

To handle this I am using two nodes
- A distance dection node, wich is a bit of the smart part identifying where to to go and keeping track of distnace
-- It is populating following event topics
--- distance the closest distance to an obstacle
--- direction a proposed direction based on the closest obstacle
--- frontdistance (obviuous)
--- leftdistance (obvious)
--- rightdistance (obvious)
- Second node is autodrive that controlles the motors and subscribes to distance and direction topics
-- Distance information is used to controle the speed
-- Direction is the guidance for which direction to drive to


Click tp see the video when it tries to avoid crashing
[![Watch the video](image_car2/robot_car2_front.jpeg)](https://youtu.be/d5yBfNilb-g)
Below is some good pictures
[![Watch the video](image_car2/IMG_2048.jpeg)](image_car2/IMG_2048.jpeg)
[![Watch the video](image_car2/robot_car2_front.jpeg)](image_car2/robot_car2_front.jpeg)
[![Watch the video](image_car2/IMG_2049.jpeg)](image_car2/IMG_2049.jpeg)
[![Watch the video](image_car2/IMG_2050.jpeg)](image_car2/IMG_2050.jpeg)
[![Watch the video](https://youtu.be/d5yBfNilb-g)](https://youtu.be/d5yBfNilb-g)


